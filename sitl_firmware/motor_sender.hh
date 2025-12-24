#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>

#include <gz/transport/Node.hh>   // <-- IMPORTANT: no forward declare!
#include <gz/msgs/actuators.pb.h>

class MotorSender {
public:
    MotorSender();
    ~MotorSender();

    void start();
    void stop();

    // Expected motor command range: [0.0 .. 1.0]
    void setMotors(const std::array<float, 4>& motors);

private:
    void run();


    std::atomic<bool> running_{false};
    std::thread worker_;

    std::mutex mtx_;
    std::array<float, 4> motors_{0.f, 0.f, 0.f, 0.f};

    std::unique_ptr<gz::transport::Node> node_;
};
