#pragma once

#include <mutex>
#include <memory>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

// UnitreeJoystick implementation that receives gamepad state via DDS instead of
// reading /dev/input/jsX. Enabled with `joystick_type: "dds"` in config.yaml so
// remote/headless setups can drive the sim's FSM-facing wireless_remote bytes
// without a physical gamepad.
class DDSJoystick : public unitree::common::UnitreeJoystick
{
public:
    explicit DDSJoystick(const std::string& topic = "rt/wirelesscontroller")
    {
        // Disable the Axis exponential smoothing — our input is already digital
        // (bits) or precise (stick floats), so smoothing would just delay the
        // L2/R2 threshold crossing past the rising edge of paired buttons.
        LT.smooth = 1.0f; RT.smooth = 1.0f;
        lx.smooth = 1.0f; ly.smooth = 1.0f; rx.smooth = 1.0f; ry.smooth = 1.0f;

        using WC = unitree_go::msg::dds_::WirelessController_;
        subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<WC>>(topic);
        subscriber_->InitChannel(
            [this](const void* m) { this->onMessage(*static_cast<const WC*>(m)); },
            1);
    }

    void update() override {}

private:
    void onMessage(const unitree_go::msg::dds_::WirelessController_& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        const uint16_t k = msg.keys();
        R1_state_ = (k >> 0) & 1;
        L1_state_ = (k >> 1) & 1;
        start((k >> 2) & 1);
        back((k >> 3) & 1);
        // L2/R2 are axes on UnitreeJoystick: emulate "fully pressed" as 1.0
        RT((k >> 4) & 1 ? 1.0f : 0.0f);
        LT((k >> 5) & 1 ? 1.0f : 0.0f);
        F1((k >> 6) & 1);
        F2((k >> 7) & 1);
        A((k >> 8) & 1);
        B((k >> 9) & 1);
        X((k >> 10) & 1);
        Y((k >> 11) & 1);
        up((k >> 12) & 1);
        right((k >> 13) & 1);
        down((k >> 14) & 1);
        left((k >> 15) & 1);
        // LB/RB on UnitreeJoystick are mapped from L1/R1 bits (see XBoxJoystick: LB <- L1)
        LB(L1_state_);
        RB(R1_state_);
        lx(msg.lx());
        ly(msg.ly());
        rx(msg.rx());
        ry(msg.ry());
    }

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>> subscriber_;
    std::mutex mtx_;
    bool R1_state_{false};
    bool L1_state_{false};
};
