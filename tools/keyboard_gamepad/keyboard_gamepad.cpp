// keyboard_gamepad: publishes Unitree WirelessController_ messages on
// rt/wirelesscontroller from keyboard input, so headless setups can drive the
// sim's FSM without a physical gamepad.
//
// Use with simulate/config.yaml: use_joystick: 1, joystick_type: "dds".
//
// Layout (Xbox-style):
//   A B X Y         -> a b x y
//   up/down/left/right -> arrow keys
//   L1 R1           -> q e
//   L2 R2           -> z c
//   start select    -> Enter Backspace
//   left stick lx,ly -> i/k (ly+/-), j/l (lx-/+)
//   right stick rx   -> u/o (rx-/+)
//   reset all       -> space
//   quit            -> Esc
//
// Pulse semantics: each button press is held for ~250ms before auto-release
// so the FSM (sampling at 1 kHz) reliably sees a rising edge.

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

using WC = unitree_go::msg::dds_::WirelessController_;

namespace bits {
constexpr uint16_t R1     = 1 << 0;
constexpr uint16_t L1     = 1 << 1;
constexpr uint16_t START  = 1 << 2;
constexpr uint16_t SELECT = 1 << 3;
constexpr uint16_t R2     = 1 << 4;
constexpr uint16_t L2     = 1 << 5;
constexpr uint16_t A      = 1 << 8;
constexpr uint16_t B      = 1 << 9;
constexpr uint16_t X      = 1 << 10;
constexpr uint16_t Y      = 1 << 11;
constexpr uint16_t UP     = 1 << 12;
constexpr uint16_t RIGHT  = 1 << 13;
constexpr uint16_t DOWN   = 1 << 14;
constexpr uint16_t LEFT   = 1 << 15;
}

struct State {
    std::mutex mtx;
    uint16_t keys = 0;
    float lx = 0, ly = 0, rx = 0, ry = 0;
    std::map<uint16_t, std::chrono::steady_clock::time_point> button_release_at;
};

static struct termios g_old_term;
static std::atomic<bool> g_quit{false};

void restore_term() { tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term); }
void set_raw_term() {
    tcgetattr(STDIN_FILENO, &g_old_term);
    std::atexit(restore_term);
    struct termios t = g_old_term;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void on_sigint(int) { g_quit = true; }

void press(State& s, uint16_t bit, int hold_ms = 250) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.keys |= bit;
    s.button_release_at[bit] = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
}

// FSM macros need the modifier (L1/R1/L2/R2) to be "pressed" BEFORE the trigger
// button rising edge, otherwise the controller's axis-smoothing on L2/R2 hasn't
// crossed the press threshold yet. Press modifiers immediately, then trigger
// buttons after MOD_PRE_ROLL_MS.
constexpr int MOD_PRE_ROLL_MS = 300;
constexpr int MOD_TOTAL_MS    = 600;
constexpr int TRIG_HOLD_MS    = 250;
constexpr uint16_t MOD_MASK = (1u<<0)|(1u<<1)|(1u<<4)|(1u<<5); // R1|L1|R2|L2

void press_combo(State& s, uint16_t combo) {
    const uint16_t mods    = combo & MOD_MASK;
    const uint16_t triggers = combo & ~MOD_MASK;
    if (mods) press(s, mods, MOD_TOTAL_MS);
    if (!triggers) return;
    // schedule the trigger press after MOD_PRE_ROLL_MS
    std::thread([&s, triggers] {
        std::this_thread::sleep_for(std::chrono::milliseconds(MOD_PRE_ROLL_MS));
        press(s, triggers, TRIG_HOLD_MS);
    }).detach();
}

void clamp_axis(float& v) { if (v > 1) v = 1; if (v < -1) v = -1; }

void axis_bump(State& s, char which, float delta) {
    std::lock_guard<std::mutex> lk(s.mtx);
    switch (which) {
        case 'x': s.lx += delta; clamp_axis(s.lx); break;
        case 'y': s.ly += delta; clamp_axis(s.ly); break;
        case 'X': s.rx += delta; clamp_axis(s.rx); break;
        case 'Y': s.ry += delta; clamp_axis(s.ry); break;
    }
}

void axes_zero(State& s) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.lx = s.ly = s.rx = s.ry = 0;
    s.keys = 0;
    s.button_release_at.clear();
}

void print_help() {
    std::cout << "\r\n=== keyboard_gamepad: publishing rt/wirelesscontroller ===\r\n"
              << "Buttons: a/b/x/y  arrows  q=L1 e=R1  z=L2 c=R2  Enter=start Backspace=select\r\n"
              << "Sticks:  i/k = ly +/-   j/l = lx -/+   u/o = rx -/+\r\n"
              << "Macros (FSM transitions for unitree_rl_mjlab g1):\r\n"
              << "  1 = L2+up   (Passive -> FixStand)\r\n"
              << "  2 = R2+A    (FixStand -> Velocity)\r\n"
              << "  3 = R1+A    (Velocity -> Mimic_Dance1)\r\n"
              << "  0 = L2+B    (any RL -> Passive)\r\n"
              << "Misc:   space=zero  ?=help  Esc/q-once-twice=quit\r\n\r\n"
              << std::flush;
}

int main(int argc, char** argv) {
    std::string iface = "lo";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-n" || a == "--network") iface = argv[++i];
        else if (a == "-h" || a == "--help") { print_help(); return 0; }
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelPublisher<WC> publisher("rt/wirelesscontroller");
    publisher.InitChannel();

    State s;
    set_raw_term();
    signal(SIGINT, on_sigint);

    print_help();

    // Publisher thread: sends state at 50 Hz, also clears expired button pulses.
    std::thread pub_thread([&] {
        WC msg;
        auto period = std::chrono::milliseconds(20);
        while (!g_quit) {
            {
                std::lock_guard<std::mutex> lk(s.mtx);
                auto now = std::chrono::steady_clock::now();
                for (auto it = s.button_release_at.begin(); it != s.button_release_at.end(); ) {
                    if (now >= it->second) { s.keys &= ~it->first; it = s.button_release_at.erase(it); }
                    else ++it;
                }
                msg.lx() = s.lx;
                msg.ly() = s.ly;
                msg.rx() = s.rx;
                msg.ry() = s.ry;
                msg.keys() = s.keys;
            }
            publisher.Write(msg);
            std::this_thread::sleep_for(period);
        }
    });

    int last_q_count = 0;
    while (!g_quit) {
        unsigned char c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }

        if (c == 27) { // Esc or escape sequence (arrows)
            unsigned char c2 = 0, c3 = 0;
            // try to read 2 more chars for arrow seqs (non-blocking)
            int n2 = read(STDIN_FILENO, &c2, 1);
            int n3 = (n2 > 0) ? read(STDIN_FILENO, &c3, 1) : 0;
            if (n2 > 0 && c2 == '[' && n3 > 0) {
                switch (c3) {
                    case 'A': press(s, bits::UP);    continue;
                    case 'B': press(s, bits::DOWN);  continue;
                    case 'C': press(s, bits::RIGHT); continue;
                    case 'D': press(s, bits::LEFT);  continue;
                }
            }
            std::cout << "\r\nbye\r\n";
            break;
        }

        switch (c) {
            case 'a': press(s, bits::A); break;
            case 'b': press(s, bits::B); break;
            case 'x': press(s, bits::X); break;
            case 'y': press(s, bits::Y); break;
            case 'q': press(s, bits::L1); break;
            case 'e': press(s, bits::R1); break;
            case 'z': press(s, bits::L2); break;
            case 'c': press(s, bits::R2); break;
            case '\n': press(s, bits::START); break;
            case 127: press(s, bits::SELECT); break; // backspace
            case 'i': axis_bump(s, 'y', +0.1f); break;
            case 'k': axis_bump(s, 'y', -0.1f); break;
            case 'j': axis_bump(s, 'x', -0.1f); break;
            case 'l': axis_bump(s, 'x', +0.1f); break;
            case 'u': axis_bump(s, 'X', -0.1f); break;
            case 'o': axis_bump(s, 'X', +0.1f); break;
            case ' ': axes_zero(s); std::cout << "\r[zeroed]\r\n"; break;
            case '?': print_help(); break;
            case '1': press_combo(s, bits::L2 | bits::UP); std::cout << "\r[macro] L2+up -> FixStand\r\n"; break;
            case '2': press_combo(s, bits::R2 | bits::A);  std::cout << "\r[macro] R2+A -> Velocity\r\n"; break;
            case '3': press_combo(s, bits::R1 | bits::A);  std::cout << "\r[macro] R1+A -> Mimic_Dance1\r\n"; break;
            case '0': press_combo(s, bits::L2 | bits::B);  std::cout << "\r[macro] L2+B -> Passive\r\n"; break;
            default: break;
        }
        (void)last_q_count;
    }

    g_quit = true;
    pub_thread.join();
    return 0;
}
