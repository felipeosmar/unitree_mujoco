// pulse_gamepad: publishes a single button-combo on rt/wirelesscontroller for
// a fixed duration, then exits. Used to verify the DDS path without TTY tricks.
//
// Usage: pulse_gamepad <KEYS_BITMASK_HEX> <DURATION_SEC> [iface]
//   e.g.  pulse_gamepad 0x1020 1   # L2 (bit5) + up (bit12) for 1s

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using WC = unitree_go::msg::dds_::WirelessController_;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <KEYS_BITMASK_HEX> <DURATION_SEC> [iface]\n", argv[0]);
        return 2;
    }
    uint16_t keys = static_cast<uint16_t>(std::strtoul(argv[1], nullptr, 0));
    double duration = std::strtod(argv[2], nullptr);
    std::string iface = (argc > 3) ? argv[3] : "lo";

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelPublisher<WC> pub("rt/wirelesscontroller");
    pub.InitChannel();

    // Let DDS discovery settle before pulsing.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    WC msg;
    msg.lx() = 0; msg.ly() = 0; msg.rx() = 0; msg.ry() = 0;

    // Pre-roll modifier bits (L1/R1/L2/R2) for 250 ms before adding the trigger
    // buttons. The UnitreeJoystick on the controller side smooths L2/R2 axes
    // (smooth=0.03), so the threshold-crossing on the modifier must happen
    // BEFORE the on_pressed rising edge of the trigger button.
    constexpr uint16_t MODIFIER_MASK = (1u<<0)|(1u<<1)|(1u<<4)|(1u<<5); // R1|L1|R2|L2
    const uint16_t modifier_bits = keys & MODIFIER_MASK;
    const uint16_t trigger_bits  = keys & ~MODIFIER_MASK;

    auto start = std::chrono::steady_clock::now();
    auto pre_end = start + std::chrono::milliseconds(modifier_bits && trigger_bits ? 250 : 0);
    auto end = start + std::chrono::milliseconds(int(duration * 1000) + (modifier_bits && trigger_bits ? 250 : 0));
    int sent = 0;
    while (std::chrono::steady_clock::now() < end) {
        msg.keys() = (std::chrono::steady_clock::now() < pre_end) ? modifier_bits : keys;
        pub.Write(msg);
        ++sent;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // tail-zero
    msg.keys() = 0;
    for (int i = 0; i < 5; ++i) {
        pub.Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    fprintf(stderr, "pulse_gamepad: sent %d frames with keys=0x%04x for %.2fs\n",
            sent, keys, duration);
    return 0;
}
