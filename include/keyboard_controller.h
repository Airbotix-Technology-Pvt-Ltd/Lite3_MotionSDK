/**
 * @file keyboard_controller.h
 */

#pragma once

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>

class KeyboardController {
public:
    static constexpr float MAX_VX = 0.30f;
    static constexpr float MAX_VY = 0.15f;
    static constexpr float MAX_WZ = 0.60f;
    static constexpr int   KEY_TIMEOUT_MS = 600;

    KeyboardController() {
        // Open /dev/tty directly — bypasses stdin redirection in Docker
        tty_fd_ = open("/dev/tty", O_RDWR | O_NOCTTY);
        if (tty_fd_ < 0) {
            tty_fd_ = STDIN_FILENO; // fallback
        }

        // Save current terminal settings
        tcgetattr(tty_fd_, &old_term_);

        // Set raw mode: no line buffering, no echo
        // Keep ISIG so Ctrl+C (SIGINT) still works!
        struct termios raw = old_term_;
        raw.c_lflag &= ~(ICANON | ECHO); 
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(tty_fd_, TCSAFLUSH, &raw);

        // Non-blocking reads
        int flags = fcntl(tty_fd_, F_GETFL, 0);
        fcntl(tty_fd_, F_SETFL, flags | O_NONBLOCK);

        last_key_time_ = std::chrono::steady_clock::now();
        running_ = true;
        thread_ = std::thread(&KeyboardController::Loop, this);

        PrintHelp();
    }

    ~KeyboardController() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        tcsetattr(tty_fd_, TCSAFLUSH, &old_term_);
        if (tty_fd_ != STDIN_FILENO) close(tty_fd_);
    }

    float vx() const { return vx_.load(); }
    float vy() const { return vy_.load(); }
    float wz() const { return wz_.load(); }
    float speed_scale() const { return speed_.load(); }
    bool  stair_mode()  const { return stair_.load(); }
    float pitch_offset() const { return pitch_off_.load(); }

private:
    int            tty_fd_{-1};
    struct termios old_term_{};
    std::thread    thread_;
    std::atomic<bool>  running_{false};
    std::atomic<float> vx_{0.f}, vy_{0.f}, wz_{0.f};
    std::atomic<float> speed_{1.0f};
    std::atomic<float> pitch_off_{0.f};
    std::atomic<bool>  stair_{false};
    std::chrono::steady_clock::time_point last_key_time_;

    void Loop() {
        while (running_) {
            char c = 0;
            if (read(tty_fd_, &c, 1) > 0) {
                last_key_time_ = std::chrono::steady_clock::now();
                HandleKey(c);
            } else {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_key_time_).count();
                if (elapsed_ms > KEY_TIMEOUT_MS) {
                    vx_.store(0.f);
                    vy_.store(0.f);
                    wz_.store(0.f);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void HandleKey(char c) {
        switch (c) {
            case 'w': case 'W':
                vx_.store(+MAX_VX); wz_.store(0.f);
                fprintf(stderr, "[KEY] W → vx=+%.2f (×%.1f)\n", MAX_VX, speed_.load()); break;
            case 's': case 'S':
                vx_.store(-MAX_VX); wz_.store(0.f);
                fprintf(stderr, "[KEY] S → vx=-%.2f (×%.1f)\n", MAX_VX, speed_.load()); break;
            case 'a': case 'A':
                wz_.store(+MAX_WZ); vx_.store(0.f);
                fprintf(stderr, "[KEY] A → wz=+%.2f\n", MAX_WZ); break;
            case 'd': case 'D':
                wz_.store(-MAX_WZ); vx_.store(0.f);
                fprintf(stderr, "[KEY] D → wz=-%.2f\n", MAX_WZ); break;
            case 'q': case 'Q':
                vy_.store(+MAX_VY);
                fprintf(stderr, "[KEY] Q → vy=+%.2f\n", MAX_VY); break;
            case 'e': case 'E':
                vy_.store(-MAX_VY);
                fprintf(stderr, "[KEY] E → vy=-%.2f\n", MAX_VY); break;
            case '+': case '=': {
                float s = std::min(speed_.load() + 0.5f, 3.0f);
                speed_.store(s);
                fprintf(stderr, "[KEY] + → speed=×%.1f\n", s); break;
            }
            case '-': case '_': {
                float s = std::max(speed_.load() - 0.5f, 0.5f);
                speed_.store(s);
                fprintf(stderr, "[KEY] - → speed=×%.1f\n", s); break;
            }
            case 't': case 'T': {
                bool now = !stair_.load();
                stair_.store(now);
                fprintf(stderr, "[KEY] T → mode=%s\n", now ? "STAIR" : "NORMAL"); break;
            }
            case 'f': case 'F': {
                float p = std::min(pitch_off_.load() + 0.05f, 0.40f);
                pitch_off_.store(p);
                fprintf(stderr, "[KEY] F → lean forward: %.2f rad\n", p); break;
            }
            case 'r': case 'R':
                pitch_off_.store(0.f);
                fprintf(stderr, "[KEY] R → reset pitch\n"); break;
            case ' ':
                vx_.store(0.f); vy_.store(0.f); wz_.store(0.f);
                fprintf(stderr, "[KEY] SPACE → STOP\n"); break;
            case 0x03:
                vx_.store(0.f); vy_.store(0.f); wz_.store(0.f);
                raise(SIGINT); break;
        }
    }

    static void PrintHelp() {
        fprintf(stderr, "\n"
            "╔══════════════════════════════╗\n"
            "║   Lite3 Keyboard Control     ║\n"
            "╠══════════════════════════════╣\n"
            "║  W / S  — Forward / Back     ║\n"
            "║  A / D  — Turn Left/Right    ║\n"
            "║  Q / E  — Strafe Left/Right  ║\n"
            "║  + / -  — Speed Up/Down      ║\n"
            "║  T      — Crawl (Stair) Mode ║\n"
            "║  F / R  — Forward Lean/Reset ║\n"
            "║  Space  — Stop               ║\n"
            "║  Ctrl+C — Quit               ║\n"
            "╚══════════════════════════════╝\n\n");
    }
};
