#pragma once

#ifdef __cplusplus
extern "C" {
#endif

static const float dt = 0.002;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_rate;
    float prev_error;

    float d_filter;
    float d_alpha;

    float out_limit;
    float i_limit;

    unsigned long last_update_us;

} PID_t;

/**
 * Initialize PID controller
 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float d_alpha,
              float out_limit,
              float i_limit);

/**
 * Update PID (rate controller)
 */
float PID_Update(PID_t *pid,
                 float setpoint,
                 float rate);

/**
 * Reset internal PID state
 */
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif
