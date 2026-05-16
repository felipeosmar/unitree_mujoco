// keyboard_platform: interactive driver for the sim's PlatformController in
// "remote" mode. Publishes rt/platform_cmd at 50 Hz with the current target
// pitch/roll (in degrees) and max rate (deg/s). Also subscribes to
// rt/platform_state to display the actual platform pose.
//
// Keys:
//   w / s        -> pitch  -/+  (deg, increment 1°)
//   a / d        -> roll   -/+
//   shift+W/S/A/D -> 5° step
//   [ / ]        -> max rate -/+ (10 deg/s steps)
//   space        -> zero target (pitch=roll=0)
//   r            -> reset rate to default 30 deg/s
//   q            -> quit

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

#include <signal.h>
#include <termios.h>
#include <unistd.h>

using WC = unitree_go::msg::dds_::WirelessController_;

static struct termios g_old_term;
static std::atomic<bool> g_quit{false};
static void restore_term() { tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term); }
static void on_sigint(int) { g_quit = true; }
static void set_raw_term() {
    tcgetattr(STDIN_FILENO, &g_old_term);
    std::atexit(restore_term);
    struct termios t = g_old_term;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

struct Target {
    std::mutex mtx;
    float pitch_deg = 0;
    float roll_deg  = 0;
    float rate_deg_s = 30;
};

struct StateView {
    std::mutex mtx;
    float pitch_deg = 0, roll_deg = 0;
    float pitch_rate = 0, roll_rate = 0;
    bool fresh = false;
};

static void print_help() {
    std::cout << "\r\n=== keyboard_platform — publishing rt/platform_cmd ===\r\n"
              << "w/s = pitch -/+ 1°    SHIFT (W/S) = 5°\r\n"
              << "a/d = roll  -/+ 1°    SHIFT (A/D) = 5°\r\n"
              << "[ / ]  = max rate -/+ 10 deg/s\r\n"
              << "space  = zero target    r = reset rate to 30\r\n"
              << "?      = help            q / Esc = quit\r\n\r\n" << std::flush;
}

int main(int argc, char** argv) {
    std::string iface = "lo";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-n" || a == "--network") iface = argv[++i];
        else if (a == "-h" || a == "--help") { print_help(); return 0; }
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelPublisher<WC> cmd_pub("rt/platform_cmd");
    cmd_pub.InitChannel();
    unitree::robot::ChannelSubscriber<WC> state_sub("rt/platform_state");

    Target tgt;
    StateView state;
    state_sub.InitChannel([&state](const void* m) {
        auto* wc = static_cast<const WC*>(m);
        std::lock_guard<std::mutex> lk(state.mtx);
        state.pitch_deg = wc->lx();
        state.roll_deg  = wc->ly();
        state.pitch_rate = wc->rx();
        state.roll_rate  = wc->ry();
        state.fresh = true;
    }, 1);

    set_raw_term();
    signal(SIGINT, on_sigint);
    print_help();

    std::thread pub_thread([&] {
        WC msg;
        while (!g_quit) {
            {
                std::lock_guard<std::mutex> lk(tgt.mtx);
                msg.lx() = tgt.pitch_deg;
                msg.ly() = tgt.roll_deg;
                msg.rx() = tgt.rate_deg_s;
                msg.ry() = 0;
                msg.keys() = 0;
            }
            cmd_pub.Write(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    auto last_print = std::chrono::steady_clock::now();

    auto bump = [&](float dp, float dr) {
        std::lock_guard<std::mutex> lk(tgt.mtx);
        tgt.pitch_deg += dp;
        tgt.roll_deg  += dr;
    };
    auto zero = [&] {
        std::lock_guard<std::mutex> lk(tgt.mtx);
        tgt.pitch_deg = 0; tgt.roll_deg = 0;
    };
    auto rate_bump = [&](float dr) {
        std::lock_guard<std::mutex> lk(tgt.mtx);
        tgt.rate_deg_s = std::max(1.f, tgt.rate_deg_s + dr);
    };

    while (!g_quit) {
        unsigned char c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n > 0) {
            switch (c) {
                case 'w': bump(+1, 0); break;
                case 's': bump(-1, 0); break;
                case 'a': bump(0, -1); break;
                case 'd': bump(0, +1); break;
                case 'W': bump(+5, 0); break;
                case 'S': bump(-5, 0); break;
                case 'A': bump(0, -5); break;
                case 'D': bump(0, +5); break;
                case '[': rate_bump(-10); break;
                case ']': rate_bump(+10); break;
                case ' ': zero(); break;
                case 'r': { std::lock_guard<std::mutex> lk(tgt.mtx); tgt.rate_deg_s = 30; } break;
                case '?': print_help(); break;
                case 'q':
                case 27: g_quit = true; break;
                default: break;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_print > std::chrono::milliseconds(200)) {
            last_print = now;
            float tp, tr, trate;
            { std::lock_guard<std::mutex> lk(tgt.mtx);
              tp=tgt.pitch_deg; tr=tgt.roll_deg; trate=tgt.rate_deg_s; }
            float ap=0, ar=0; bool fresh=false;
            { std::lock_guard<std::mutex> lk(state.mtx);
              ap=state.pitch_deg; ar=state.roll_deg; fresh=state.fresh; }
            std::printf("\r target pitch=%+6.1f° roll=%+6.1f° rate=%4.0f°/s  | actual %s%+6.1f° %+6.1f° \033[K",
                        tp, tr, trate, fresh ? "" : "(no state) ", ap, ar);
            std::fflush(stdout);
        }

        if (n <= 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "\r\nbye\r\n";
    g_quit = true;
    pub_thread.join();
    return 0;
}
