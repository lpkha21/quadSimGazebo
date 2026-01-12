#include "motor_sender.hh"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

constexpr float MOTOR_MIN_RAD_S = 0.0f;
constexpr float MOTOR_HOVER_RAD_S = 830.0f;
constexpr float MOTOR_MAX_RAD_S = 1500.0f;

MotorSender::MotorSender()
  : node_(std::make_unique<gz::transport::Node>())
{
}

MotorSender::~MotorSender()
{
    stop();
}

void MotorSender::start()
{
    if (running_) return;
    running_ = true;
    worker_ = std::thread(&MotorSender::run, this);
}

void MotorSender::stop()
{
    running_ = false;
    if (worker_.joinable())
        worker_.join();
}

void MotorSender::setMotors(const std::array<float, 4>& motors)
{
    std::lock_guard<std::mutex> lk(mtx_);
    motors_ = motors;
}

float thrust_to_speed(float u)
{
    // u ∈ [0..1]
    return MOTOR_MIN_RAD_S +
           u * (MOTOR_MAX_RAD_S - MOTOR_MIN_RAD_S);
}

void MotorSender::run()
{
    // Topic: adjust to match your world / model name
    auto pub = node_->Advertise<gz::msgs::Actuators>(
        "/x500/command/motor_speed"
    );

    gz::msgs::Actuators msg;
    std::array<float, 4> local{};

    while (running_) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            local = motors_;
        }

        msg.mutable_velocity()->Clear();
        for (int i = 0; i < 4; i++) {
            float u = std::clamp(local[i], 0.0f, 1.0f);
            float omega = thrust_to_speed(u);
            msg.mutable_velocity()->Add(omega);
        }

        pub.Publish(msg);

        // ~400-500 Hz is typical (PX4 publishes around 400 Hz)
        std::this_thread::sleep_for(2ms);
    }
}
