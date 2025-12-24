#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <memory>

// Gazebo transport + IMU message
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>

struct ImuSample {
    double stamp_sec = 0.0;
    float gx = 0.0f; // rad/s
    float gy = 0.0f; // rad/s
    float gz = 0.0f; // rad/s
};

class ImuReceiver {
public:
    ImuReceiver();
    ~ImuReceiver();

    bool start(const std::string& topic = "/imu");
    void stop();

    bool getLatest(ImuSample& out) const;

private:
    void onImuMsg(const gz::msgs::IMU& msg);

    std::atomic<bool> running_{false};
    mutable std::mutex mtx_;
    ImuSample latest_{};
    bool have_{false};

    std::unique_ptr<gz::transport::Node> node_;
};
