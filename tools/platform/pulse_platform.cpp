// pulse_platform: publish a single WirelessController_-encoded platform target
// on rt/platform_cmd (consumed by the sim's PlatformController in mode "remote").
//
// Usage: pulse_platform <pitch_deg> <roll_deg> [max_rate_deg_s] [duration_s] [iface]
//   pitch positive = nose-up tilt around X axis
//   roll  positive = right-side-up tilt around Y axis
//   max_rate_deg_s = 0 keeps the default from config.yaml
//   duration_s defaults to 0.5
//   iface defaults to "lo"

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
        std::fprintf(stderr,
            "usage: %s <pitch_deg> <roll_deg> [max_rate_deg_s=0] [duration_s=0.5] [iface=lo]\n",
            argv[0]);
        return 2;
    }
    float pitch_deg = std::strtof(argv[1], nullptr);
    float roll_deg  = std::strtof(argv[2], nullptr);
    float rate_deg_s = (argc > 3) ? std::strtof(argv[3], nullptr) : 0.f;
    double duration = (argc > 4) ? std::strtod(argv[4], nullptr) : 0.5;
    std::string iface = (argc > 5) ? argv[5] : "lo";

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelPublisher<WC> pub("rt/platform_cmd");
    pub.InitChannel();

    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // DDS discovery

    WC msg;
    msg.lx() = pitch_deg;
    msg.ly() = roll_deg;
    msg.rx() = rate_deg_s;
    msg.ry() = 0;
    msg.keys() = 0;

    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(int(duration * 1000));
    int sent = 0;
    while (std::chrono::steady_clock::now() < end) {
        pub.Write(msg);
        ++sent;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::fprintf(stderr, "pulse_platform: held pitch=%.2f° roll=%.2f° rate=%.2f for %.2fs (%d frames)\n",
                 pitch_deg, roll_deg, rate_deg_s, duration, sent);
    return 0;
}
