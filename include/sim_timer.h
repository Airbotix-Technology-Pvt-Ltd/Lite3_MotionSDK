/**
 * @file sim_timer.h
 * @brief A simple real-time timer for controlling loop frequency.
 *        Mirrors MotionSDK's DRTimer API so existing gait code is easy to port.
 *
 * Usage:
 *   SimTimer timer;
 *   timer.TimeInit(1);                     // 1 ms period
 *   double t0 = timer.GetCurrentTime();
 *   while(true) {
 *       if (timer.TimerInterrupt()) continue; // not time yet
 *       double t = timer.GetIntervalTime(t0);
 *       // ... your gait logic here ...
 *   }
 */

#pragma once

#include <chrono>
#include <ctime>
#include <thread>

class SimTimer {
public:
    /**
     * @brief Initialize the timer.
     * @param period_ms  Control loop period in milliseconds (e.g. 1 for 1 kHz)
     */
    void TimeInit(int period_ms) {
        period_ns_ = static_cast<long>(period_ms) * 1'000'000L;
        last_tick_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Get absolute time in seconds (equivalent to DRTimer::GetCurrentTime).
     */
    double GetCurrentTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    /**
     * @brief Elapsed seconds since start_time (equivalent to DRTimer::GetIntervalTime).
     */
    double GetIntervalTime(double start_time) {
        return GetCurrentTime() - start_time;
    }

    /**
     * @brief Waits until the next period then returns false (= time to run).
     *        Returns true while still within the current period (= skip).
     *        Mirrors DRTimer::TimerInterrupt().
     */
    bool TimerInterrupt() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now - last_tick_).count();

        if (elapsed_ns < period_ns_) {
            // Sleep a bit to avoid busy-waiting 100% CPU
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            return true;   // "interrupt still pending" — skip
        }

        last_tick_ = now;
        return false;  // time to execute
    }

private:
    long period_ns_{1'000'000}; // default 1 ms
    std::chrono::steady_clock::time_point last_tick_;
};
