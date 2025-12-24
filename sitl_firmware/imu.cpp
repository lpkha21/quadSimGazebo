#include "imu.hh"

#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>

ImuReceiver::ImuReceiver()
  : node_(std::make_unique<gz::transport::Node>())
{}

ImuReceiver::~ImuReceiver() { stop(); }

bool ImuReceiver::start(const std::string& topic)
{
    if (running_) return true;
    running_ = true;

    // Subscribe IMU
    bool ok = node_->Subscribe(topic, &ImuReceiver::onImuMsg, this);
    return ok;
}

void ImuReceiver::stop()
{
    running_ = false;
}

void ImuReceiver::onImuMsg(const gz::msgs::IMU& msg)
{
    ImuSample s;
    // timestamp (sec)
    s.stamp_sec = static_cast<double>(msg.header().stamp().sec()) +
                  static_cast<double>(msg.header().stamp().nsec()) * 1e-9;

    // Angular velocity (rad/s)
    s.gx = static_cast<float>(msg.angular_velocity().x());
    s.gy = static_cast<float>(msg.angular_velocity().y());
    s.gz = -static_cast<float>(msg.angular_velocity().z()); // TODO: YAW INVERTED

    std::lock_guard<std::mutex> lk(mtx_);
    latest_ = s;
    have_ = true;
}

bool ImuReceiver::getLatest(ImuSample& out) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_) return false;
    out = latest_;
    return true;
}
