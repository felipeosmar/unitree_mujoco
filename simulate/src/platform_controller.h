#pragma once

// PlatformController — drives the optional tilting platform declared in
// scene_platform.xml. The platform is a MOCAP body, so its world pose is
// written directly to data->mocap_pos / mocap_quat each physics step; it
// does NOT participate in dynamics. The robot can't push the platform any
// more than a real boat is pushed by its passenger — the boat moves with the
// water, not with whoever is standing on it. This eliminates the PD-vs-mass
// feedback loop that destabilised the previous hinge+PD design.
//
// Modes (config.yaml):
//   "off"    — platform stays level at 0,0
//   "remote" — target pitch/roll come from rt/platform_cmd (a
//              WirelessController_ where lx=pitch_deg, ly=roll_deg,
//              rx=max_rate_deg_s; rx=0 means use default from yaml)
//   "boat"   — server-side sine wave:
//                target_pitch(t) = ramp(t) · A_p · sin(2π t / T_p)
//                target_roll(t)  = ramp(t) · A_r · sin(2π t / T_r + φ)
//              where ramp(t) = clamp(t / rampup_s, 0, 1) so the motion fades
//              in from zero over rampup_s seconds.
//
// State is published on rt/platform_state @ 50 Hz (WirelessController_:
// lx=actual pitch_deg, ly=actual roll_deg, rx/ry=angular velocities deg/s).

#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include "param.h"

namespace platform_dds {
inline constexpr const char* kCmdTopic   = "rt/platform_cmd";
inline constexpr const char* kStateTopic = "rt/platform_state";
}  // namespace platform_dds

class PlatformController
{
public:
    PlatformController(mjModel* model, mjData* data) : m_(model), d_(data)
    {
        const int body_id = mj_name2id(m_, mjOBJ_BODY, "platform");
        if (body_id < 0) {
            active_ = false;
            return;
        }
        mocap_id_ = m_->body_mocapid[body_id];
        if (mocap_id_ < 0) {
            std::cerr << "PlatformController: body 'platform' exists but is not mocap; bailing out\n";
            active_ = false;
            return;
        }
        // Anchor position of the platform (the mocap_pos that we keep fixed).
        anchor_pos_[0] = m_->body_pos[3*body_id + 0];
        anchor_pos_[1] = m_->body_pos[3*body_id + 1];
        anchor_pos_[2] = m_->body_pos[3*body_id + 2];

        active_ = true;
        const std::string& m = param::config.platform_mode;
        if (m != "off" && m != "remote" && m != "boat") {
            std::cerr << "PlatformController: unknown platform_mode '" << m
                      << "', falling back to 'off'\n";
            mode_ = "off";
        } else {
            mode_ = m;
        }
        max_rate_deg_s_ = param::config.platform_default_max_rate_deg_s;

        using WC = unitree_go::msg::dds_::WirelessController_;
        state_pub_ = std::make_unique<unitree::robot::ChannelPublisher<WC>>(platform_dds::kStateTopic);
        state_pub_->InitChannel();

        if (mode_ == "remote") {
            cmd_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<WC>>(platform_dds::kCmdTopic);
            cmd_sub_->InitChannel([this](const void* m) {
                auto* wc = static_cast<const WC*>(m);
                std::lock_guard<std::mutex> lk(mtx_);
                target_pitch_deg_ = wc->lx();
                target_roll_deg_  = wc->ly();
                if (wc->rx() > 0.01f) max_rate_deg_s_ = wc->rx();
            }, 1);
        }

        boot_time_ = std::chrono::steady_clock::now();
        std::cout << "PlatformController: active (mocap) in '" << mode_ << "' mode "
                  << "(default_rate=" << max_rate_deg_s_ << " deg/s)" << std::endl;
    }

    bool active() const { return active_; }

    void step()
    {
        if (!active_ || mode_ == "off") {
            // Even in off mode, hold platform at 0,0 (so it doesn't sit at
            // whatever quaternion mocap was initialised with).
            if (active_) writePose(0.0, 0.0);
            return;
        }

        const double now_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - boot_time_).count();
        double dt = (last_step_time_ >= 0) ? (now_s - last_step_time_) : m_->opt.timestep;
        if (dt <= 0 || dt > 0.1) dt = m_->opt.timestep;
        last_step_time_ = now_s;

        double tgt_pitch_deg, tgt_roll_deg, rate_limit;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (mode_ == "boat") {
                const double Tp = std::max(0.1, param::config.boat_pitch_period_s);
                const double Tr = std::max(0.1, param::config.boat_roll_period_s);
                const double ramp = param::config.boat_rampup_s > 0
                    ? std::clamp(now_s / param::config.boat_rampup_s, 0.0, 1.0)
                    : 1.0;
                tgt_pitch_deg = ramp * param::config.boat_pitch_amp_deg * std::sin(2.0 * M_PI * now_s / Tp);
                tgt_roll_deg  = ramp * param::config.boat_roll_amp_deg  * std::sin(2.0 * M_PI * now_s / Tr
                                                                                  + param::config.boat_phase_offset_rad);
            } else {
                tgt_pitch_deg = target_pitch_deg_;
                tgt_roll_deg  = target_roll_deg_;
            }
            rate_limit = max_rate_deg_s_;
        }

        // Rate-limit the commanded angles. Even though the platform is mocap
        // (kinematic), abrupt jumps would create discontinuous contact velocities
        // that hit the robot like a hammer; rate-limiting keeps things smooth.
        const double max_delta = rate_limit * dt;
        cmd_pitch_deg_ += std::clamp(tgt_pitch_deg - cmd_pitch_deg_, -max_delta, max_delta);
        cmd_roll_deg_  += std::clamp(tgt_roll_deg  - cmd_roll_deg_,  -max_delta, max_delta);

        writePose(cmd_pitch_deg_, cmd_roll_deg_);

        // Publish state at ~50 Hz.
        if (++pub_counter_ * m_->opt.timestep >= 0.02) {
            pub_counter_ = 0;
            const double rate_p_deg = (cmd_pitch_deg_ - last_pub_pitch_) /
                                      std::max(0.001, now_s - last_pub_time_);
            const double rate_r_deg = (cmd_roll_deg_  - last_pub_roll_)  /
                                      std::max(0.001, now_s - last_pub_time_);
            last_pub_pitch_ = cmd_pitch_deg_;
            last_pub_roll_  = cmd_roll_deg_;
            last_pub_time_  = now_s;

            unitree_go::msg::dds_::WirelessController_ msg;
            msg.lx() = static_cast<float>(cmd_pitch_deg_);
            msg.ly() = static_cast<float>(cmd_roll_deg_);
            msg.rx() = static_cast<float>(rate_p_deg);
            msg.ry() = static_cast<float>(rate_r_deg);
            msg.keys() = 0;
            state_pub_->Write(msg);
        }
    }

private:
    void writePose(double pitch_deg, double roll_deg)
    {
        const double deg2rad = M_PI / 180.0;
        const double pitch = pitch_deg * deg2rad;
        const double roll  = roll_deg  * deg2rad;
        // Combined rotation: first pitch around X, then roll around Y.
        // q = q_roll · q_pitch  (right multiplication = roll applied second)
        const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
        // q_pitch = [cp, sp, 0, 0]   (rotation around X)
        // q_roll  = [cr, 0, sr, 0]   (rotation around Y)
        // q = q_roll * q_pitch (Hamilton product, MuJoCo uses wxyz)
        const double w = cr * cp;
        const double x = cr * sp;
        const double y = sr * cp;
        const double z = -sr * sp;

        d_->mocap_pos[3*mocap_id_ + 0] = anchor_pos_[0];
        d_->mocap_pos[3*mocap_id_ + 1] = anchor_pos_[1];
        d_->mocap_pos[3*mocap_id_ + 2] = anchor_pos_[2];
        d_->mocap_quat[4*mocap_id_ + 0] = w;
        d_->mocap_quat[4*mocap_id_ + 1] = x;
        d_->mocap_quat[4*mocap_id_ + 2] = y;
        d_->mocap_quat[4*mocap_id_ + 3] = z;
    }

    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;
    bool active_ = false;
    std::string mode_ = "off";

    int mocap_id_ = -1;
    double anchor_pos_[3] = {0, 0, 0};

    std::mutex mtx_;
    double target_pitch_deg_ = 0.0;
    double target_roll_deg_  = 0.0;
    double max_rate_deg_s_   = 30.0;
    double cmd_pitch_deg_ = 0.0;
    double cmd_roll_deg_  = 0.0;

    std::chrono::steady_clock::time_point boot_time_;
    double last_step_time_ = -1.0;
    int pub_counter_ = 0;
    double last_pub_pitch_ = 0.0;
    double last_pub_roll_  = 0.0;
    double last_pub_time_  = 0.0;

    using WC = unitree_go::msg::dds_::WirelessController_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<WC>> cmd_sub_;
    std::unique_ptr<unitree::robot::ChannelPublisher<WC>> state_pub_;
};
