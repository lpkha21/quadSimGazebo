#include <iostream>
#include <array>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>

#include "rc.hh"
#include "imu.hh"
#include "motor_sender.hh"

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
static inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }
static inline float deg2rad(float d) { return d * 0.01745329252f; }

struct RatePID {
    float kp=0, ki=0, kd=0;
    float integrator=0;
    float i_limit=0.3f;
    float prev_meas = 0.0f;
    float d_filt = 0.0f;
    float d_cutoff = 30.0f; // Hz, safe starting point

    float update(float sp, float meas, float dt) {
        float err = sp - meas;
        integrator += ki * err * dt;
        integrator = clampf(integrator, -i_limit, i_limit);

        float d_meas = (meas - prev_meas) / dt;
        prev_meas = meas;
        float rc = 1.0f / (2.0f * M_PI * d_cutoff);
        
        float alpha = dt / (dt + rc);
        d_filt += alpha * (d_meas - d_filt);

        float p = kp * err;
        float i = integrator;
        float d = -kd * d_filt;

        return p + i + d;
    }

    void reset() { 
        integrator = 0.0f; 
        d_filt = 0;
    }
};


struct LowPassFilter {
    float cutoff_hz;
    float state = 0.0f;
    bool initialized = false;

    LowPassFilter(float cutoff) : cutoff_hz(cutoff) {}

    float apply(float input, float dt) {
        if (!initialized) {
            state = input;
            initialized = true;
            return input;
        }
        float rc = 1.0f / (2.0f * M_PI * cutoff_hz);
        float alpha = dt / (dt + rc);
        state += alpha * (input - state);
        return state;
    }

    void reset(float value = 0.0f) {
        state = value;
        initialized = true;
    }
};


int main()
{
    RcInput rc;
    ImuReceiver imu;
    MotorSender motors;

    if (!rc.start()) {
        std::cerr << "RC failed\n";
        return 1;
    }
    if (!imu.start("/imu")) {
        std::cerr << "IMU failed\n";
        return 1;
    }

    motors.start(); // ensure topic = /x500/command/motor_speed

    // Rate limits
    const float MAX_ROLL_RATE  = deg2rad(180.0f);
    const float MAX_PITCH_RATE = deg2rad(180.0f);
    const float MAX_YAW_RATE   = deg2rad(200.0f);

    // PID gains (starter)
    RatePID pid_r{0.05f, 0.001f, 0.0005f};
    RatePID pid_p{0.05f, 0.001f, 0.0005f};
    RatePID pid_y{0.2f, 0.0005f,  0.0f};

    LowPassFilter gyro_lpf_x(80.0f);   // roll gyro LPF
    LowPassFilter gyro_lpf_y(80.0f);   // pitch gyro LPF
    LowPassFilter gyro_lpf_z(50.0f);   // yaw gyro LPF (lower on purpose)



    const float LOOP_HZ = 250.0f;
    const auto period = std::chrono::microseconds((int)(1e6f / LOOP_HZ));
    auto last = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        dt = clampf(dt, 0.001f, 0.02f);

        last = now;
        if (dt <= 0.0f) dt = 1.0f / LOOP_HZ;

        ImuSample s;
        if (!imu.getLatest(s)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Gyro rates (rad/s)
        float gx = gyro_lpf_x.apply(s.gx, dt);
        float gy = gyro_lpf_y.apply(s.gy, dt);
        float gz = gyro_lpf_z.apply(s.gz, dt);


        // RC → rate setpoints
        float t_stick = clamp01(rc.throttle());   // 0..1
        const float IDLE_THRUST = 0.05f;

        // motors OFF below tiny threshold, else idle + collective
        float t = (t_stick < 0.02f) ? 0.0f : (IDLE_THRUST + (1.0f - IDLE_THRUST) * t_stick);

        float roll_sp  = rc.roll()  * MAX_ROLL_RATE;
        float pitch_sp = -rc.pitch() * MAX_PITCH_RATE;
        float yaw_sp   = rc.yaw()   * MAX_YAW_RATE;

        // Reset integrators when disarmed / low throttle
        if (t < 0.05f) {
            pid_r.reset();
            pid_p.reset();
            pid_y.reset();

            gyro_lpf_x.reset(s.gx);
            gyro_lpf_y.reset(s.gy);
            gyro_lpf_z.reset(s.gz);
        }

        // Rate PID
        float u_r = pid_r.update(roll_sp,  gx, dt);
        float u_p = pid_p.update(pitch_sp, gy, dt);
        float u_y = pid_y.update(yaw_sp,   gz, dt);

        // Scale PID outputs into mixer space
        const float MIX_SCALE = 0.25f;
        float r = clampf(u_r * MIX_SCALE, -0.4f, 0.4f);
        float p = clampf(u_p * MIX_SCALE, -0.4f, 0.4f);
        float y = clampf(u_y * MIX_SCALE, -0.3f, 0.3f);

        // x500 mixer
        // [0]=FR, [1]=BL, [2]=FL, [3]=BR
        //y = 0; // TODO:YAW DISABLED
        std::array<float,4> m;
        m[0] = t - r - p + y;
        m[1] = t + r + p + y;
        m[2] = t + r - p - y;
        m[3] = t - r + p - y;

        // Normalize & clamp
        float maxv = *std::max_element(m.begin(), m.end());
        if (maxv > 1.0f) {
            for (auto &v : m) v /= maxv;
        }
        
        std::array<float,4> omega;
        for (int i=0;i<4;i++) {
            omega[i] = clamp01(m[i]);
        }
        motors.setMotors(omega);


        std::this_thread::sleep_until(now + period);
    }
}
