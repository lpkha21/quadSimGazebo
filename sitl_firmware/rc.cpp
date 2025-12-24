#include "rc.hh"

#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

RcInput::RcInput()
{
    channels_.fill(0.0f);
}

RcInput::~RcInput()
{
    stop();
}

bool RcInput::start()
{
    if (running_) return true;

    js_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js_fd_ < 0) {
        std::cerr << "RC: failed to open /dev/input/js0\n";
        return false;
    }

    running_ = true;
    worker_ = std::thread(&RcInput::run, this);
    return true;
}

void RcInput::stop()
{
    running_ = false;

    if (worker_.joinable())
        worker_.join();

    if (js_fd_ >= 0) {
        close(js_fd_);
        js_fd_ = -1;
    }
}

void RcInput::run()
{
    js_event e;

    while (running_) {
        ssize_t n = read(js_fd_, &e, sizeof(e));
        if (n != sizeof(e)) {
            usleep(1000);
            continue;
        }

        e.type &= ~JS_EVENT_INIT;

        std::lock_guard<std::mutex> lk(mtx_);

        if (e.type == JS_EVENT_AXIS && e.number < 8) {
            float v = static_cast<float>(e.value) / 32767.0f;

            // Throttle is usually inverted
            if (e.number == static_cast<int>(RcChannel::Throttle)) {
                v = (1.0f + v) * 0.5f;   // [-1..1] → [0..1]
            }

            channels_[e.number] = v;
        }

        // Buttons → Aux channels if needed
        if (e.type == JS_EVENT_BUTTON && e.number < 4) {
            channels_[4 + e.number] = e.value ? 1.0f : 0.0f;
        }
    }
}

std::array<float, 8> RcInput::getChannels() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_;
}

float RcInput::roll() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_[static_cast<int>(RcChannel::Roll)];
}

float RcInput::pitch() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_[static_cast<int>(RcChannel::Pitch)];
}

float RcInput::yaw() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_[static_cast<int>(RcChannel::Yaw)];
}

float RcInput::throttle() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_[static_cast<int>(RcChannel::Throttle)];
}
