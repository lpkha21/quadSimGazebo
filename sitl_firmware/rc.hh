#pragma once

#include <array>
#include <atomic>
#include <thread>
#include <mutex>

// Standard RC channel order (PX4-style)
enum class RcChannel : int {
    Roll     = 0,  // Aileron
    Pitch    = 1,  // Elevator
    Throttle = 2,
    Yaw      = 3,
    Aux1     = 4,
    Aux2     = 5,
    Aux3     = 6,
    Aux4     = 7
};

class RcInput {
public:
    RcInput();
    ~RcInput();

    bool start();
    void stop();

    // Get all channels (thread-safe)
    std::array<float, 8> getChannels() const;

    // Individual access helpers
    float roll() const;
    float pitch() const;
    float yaw() const;
    float throttle() const;

private:
    void run();

    std::atomic<bool> running_{false};
    std::thread worker_;

    mutable std::mutex mtx_;
    std::array<float, 8> channels_{};

    int js_fd_{-1};
};
