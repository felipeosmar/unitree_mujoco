#pragma once

// PlatformController — drives the optional dual-axis tilting platform declared
// in scene_platform.xml. Reads target pitch/roll (in degrees) either from
//   • rt/platform_cmd  (mode "remote") — message is a WirelessController_ where
//        lx = target_pitch_deg
//        ly = target_roll_deg
//        rx = max_rate_deg_s (0 -> use default from config.yaml)
//   • A server-side sine-wave generator (mode "boat") parameterised in
//     config.yaml under platform_boat:.
//
// Targets are rate-limited and applied as joint-space torques on the platform's
// pitch/roll hinges via qfrc_applied, so adding the platform doesn't change
// m->nu (which would break the G1 motor index mapping).
//
// Current platform pose is published on rt/platform_state every step using the
// same WirelessController_ IDL (lx=pitch_deg, ly=roll_deg, rx=pitch_rate_deg_s,
// ry=roll_rate_deg_s).

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
        pitch_jnt_ = mj_name2id(m_, mjOBJ_JOINT, "platform_pitch");
        roll_jnt_  = mj_name2id(m_, mjOBJ_JOINT, "platform_roll");
        if (pitch_jnt_ < 0 || roll_jnt_ < 0) {
            // Scene has no platform — controller is a no-op.
            active_ = false;
            return;
        }
        pitch_qpos_adr_ = m_->jnt_qposadr[pitch_jnt_];
        roll_qpos_adr_  = m_->jnt_qposadr[roll_jnt_];
        pitch_dof_adr_  = m_->jnt_dofadr[pitch_jnt_];
        roll_dof_adr_   = m_->jnt_dofadr[roll_jnt_];
        active_ = true;

        const std::string& mode = param::config.platform_mode;
        if (mode != "off" && mode != "remote" && mode != "boat") {
            std::cerr << "PlatformController: unknown platform_mode '" << mode
                      << "', falling back to 'off'\n";
            mode_ = "off";
        } else {
            mode_ = mode;
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
        std::cout << "PlatformController: active in '" << mode_ << "' mode "
                  << "(kp=" << param::config.platform_kp
                  << " kd=" << param::config.platform_kd
                  << " default_rate=" << max_rate_deg_s_ << " deg/s)" << std::endl;
    }

    bool active() const { return active_; }

    // Called every physics step under the sim mutex. Pulls the current target
    // (from boat generator or DDS), rate-limits, computes PD torque, writes
    // qfrc_applied. Publishes platform state on a downsampled cadence.
    void step()
    {
        if (!active_ || mode_ == "off") return;

        double now_s = std::chrono::duration<double>(
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
                tgt_pitch_deg = param::config.boat_pitch_amp_deg * std::sin(2.0 * M_PI * now_s / Tp);
                tgt_roll_deg  = param::config.boat_roll_amp_deg  * std::sin(2.0 * M_PI * now_s / Tr
                                                                            + param::config.boat_phase_offset_rad);
            } else {
                tgt_pitch_deg = target_pitch_deg_;
                tgt_roll_deg  = target_roll_deg_;
            }
            rate_limit = max_rate_deg_s_;
        }

        // Rate-limit the commanded setpoint that we feed to the PD.
        double max_delta = rate_limit * dt;
        cmd_pitch_deg_ += std::clamp(tgt_pitch_deg - cmd_pitch_deg_, -max_delta, max_delta);
        cmd_roll_deg_  += std::clamp(tgt_roll_deg  - cmd_roll_deg_,  -max_delta, max_delta);

        const double deg2rad = M_PI / 180.0;
        const double kp = param::config.platform_kp;
        const double kd = param::config.platform_kd;

        double q_pitch  = d_->qpos[pitch_qpos_adr_];
        double dq_pitch = d_->qvel[pitch_dof_adr_];
        double q_roll   = d_->qpos[roll_qpos_adr_];
        double dq_roll  = d_->qvel[roll_dof_adr_];

        d_->qfrc_applied[pitch_dof_adr_] =
            kp * (cmd_pitch_deg_ * deg2rad - q_pitch) - kd * dq_pitch;
        d_->qfrc_applied[roll_dof_adr_]  =
            kp * (cmd_roll_deg_  * deg2rad - q_roll)  - kd * dq_roll;

        // Publish state at ~50 Hz to avoid flooding DDS.
        if (++pub_counter_ * m_->opt.timestep >= 0.02) {
            pub_counter_ = 0;
            unitree_go::msg::dds_::WirelessController_ msg;
            msg.lx() = static_cast<float>(q_pitch / deg2rad);
            msg.ly() = static_cast<float>(q_roll  / deg2rad);
            msg.rx() = static_cast<float>(dq_pitch / deg2rad);
            msg.ry() = static_cast<float>(dq_roll  / deg2rad);
            msg.keys() = 0;
            state_pub_->Write(msg);
        }
    }

private:
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;
    bool active_ = false;
    std::string mode_ = "off";

    int pitch_jnt_ = -1, roll_jnt_ = -1;
    int pitch_qpos_adr_ = -1, roll_qpos_adr_ = -1;
    int pitch_dof_adr_  = -1, roll_dof_adr_  = -1;

    std::mutex mtx_;
    double target_pitch_deg_ = 0.0;
    double target_roll_deg_  = 0.0;
    double max_rate_deg_s_   = 30.0;
    double cmd_pitch_deg_ = 0.0;
    double cmd_roll_deg_  = 0.0;

    std::chrono::steady_clock::time_point boot_time_;
    double last_step_time_ = -1.0;
    int pub_counter_ = 0;

    using WC = unitree_go::msg::dds_::WirelessController_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<WC>> cmd_sub_;
    std::unique_ptr<unitree::robot::ChannelPublisher<WC>> state_pub_;
};
