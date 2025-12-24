/*
 * pid.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Pkhala
 */
#include "pid.hh"


void PID_Init(PID_t* pid, float kp, float ki, float kd, float d_alpha, float out_limit, float i_limit){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integrator = 0;
	pid->prev_rate = 0;
    pid->prev_error = 0;
    pid->d_filter = 0;
    pid->d_alpha = d_alpha;     // derivative filter coefficient (0.02–0.1 typical)
    pid->out_limit = out_limit;
    pid->i_limit = i_limit;
    pid->last_update_us = 0;
}

float PID_Update(PID_t *pid, float input, float rate) {
    float error = input - rate;
    float pTerm = pid->kp * error;


    float integrator = 0.5f * pid->ki * (error + pid->prev_error) * dt;
    float PI = pTerm + integrator;
    // Trapezoidal integration
    if(PI < pid->out_limit && PI > -pid->out_limit) {
        pid->integrator += integrator;
    }
    // clamp the integrator
    if (pid->integrator > pid->i_limit) {
        pid->integrator = pid->i_limit;
    }
    else if (pid->integrator < -pid->i_limit) {
        pid->integrator = -pid->i_limit;
    }

    // --- D TERM --- (derivative of RATE, not ERROR)
    float rate_diff = (pid->prev_rate - rate) / dt;
    
    pid->d_filter += pid->d_alpha * (rate_diff - pid->d_filter);
    float dTerm = pid->kd * pid->d_filter;
    // --- OUTPUT SUM ---
    float output = pTerm + pid->integrator + dTerm;

    // output limit
    if (output > pid->out_limit) {
        output = pid->out_limit;
    } else if (output < -pid->out_limit) {
        output = -pid->out_limit;
    }

    // slight I-term decay
    // pid->integrator *= 0.998f;

    // store previous values
    pid->prev_error = error;
    pid->prev_rate = rate;

    return output;
}

void PID_Reset(PID_t *pid) {
	pid->integrator = 0.0f;
	pid->prev_rate = 0.0f;
	pid->d_filter = 0.0f;
}
