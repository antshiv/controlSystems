#include "pid.h"

#include <math.h>
#include <string.h>

#include "attitude/quaternion.h"

static void clamp_abs(double* value, double limit) {
    if (limit <= 0.0) {
        return;
    }
    if (*value > limit) {
        *value = limit;
    } else if (*value < -limit) {
        *value = -limit;
    }
}

static void cs_quaternion_error(const double q_des[4],
                                const double q_cur[4],
                                double error_out[3]) {
    double q_des_n[4] = {q_des[0], q_des[1], q_des[2], q_des[3]};
    double q_cur_n[4] = {q_cur[0], q_cur[1], q_cur[2], q_cur[3]};
    quaternion_normalize(q_des_n);
    quaternion_normalize(q_cur_n);

    double q_cur_inv[4];
    if (!quaternion_inverse(q_cur_n, q_cur_inv)) {
        error_out[0] = error_out[1] = error_out[2] = 0.0;
        return;
    }

    double q_err[4];
    quaternion_multiply(q_des_n, q_cur_inv, q_err);

    if (q_err[0] < 0.0) {
        q_err[0] = -q_err[0];
        q_err[1] = -q_err[1];
        q_err[2] = -q_err[2];
        q_err[3] = -q_err[3];
    }

    error_out[0] = 2.0 * q_err[1];
    error_out[1] = 2.0 * q_err[2];
    error_out[2] = 2.0 * q_err[3];
}

void cs_pid_init(cs_pid_controller_t* pid, const cs_pid_gains_t* gains) {
    if (!pid || !gains) {
        return;
    }
    pid->gains = *gains;
    pid->state.integrator = 0.0;
    pid->state.prev_error = 0.0;
    pid->state.has_prev = 0;
}

void cs_pid_reset(cs_pid_controller_t* pid) {
    if (!pid) {
        return;
    }
    pid->state.integrator = 0.0;
    pid->state.prev_error = 0.0;
    pid->state.has_prev = 0;
}

double cs_pid_update(cs_pid_controller_t* pid,
                     double error,
                     double error_rate,
                     double dt) {
    if (!pid || dt <= 0.0) {
        return 0.0;
    }

    const cs_pid_gains_t* g = &pid->gains;
    cs_pid_state_t* s = &pid->state;

    double output = 0.0;

    output += g->kp * error;

    s->integrator += g->ki * error * dt;
    clamp_abs(&s->integrator, g->integrator_limit);
    output += s->integrator;

    double d_term = g->kd * error_rate;
    output += d_term;

    clamp_abs(&output, g->output_limit);

    s->prev_error = error;
    s->has_prev = 1;
    return output;
}

void cs_attitude_pid_init(cs_attitude_pid_t* ctrl,
                          const cs_pid_gains_t gains[3],
                          double rate_weight) {
    if (!ctrl || !gains) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        cs_pid_init(&ctrl->axes[i], &gains[i]);
    }
    ctrl->rate_weight = rate_weight;
}

void cs_attitude_pid_reset(cs_attitude_pid_t* ctrl) {
    if (!ctrl) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        cs_pid_reset(&ctrl->axes[i]);
    }
}

void cs_attitude_pid_update(cs_attitude_pid_t* ctrl,
                            const cs_attitude_setpoint_t* setpoint,
                            const cs_state_t* state,
                            double dt,
                            cs_actuator_command_t* out_cmd) {
    if (!ctrl || !setpoint || !state || !out_cmd || dt <= 0.0) {
        return;
    }

    double attitude_error[3];
    cs_quaternion_error(setpoint->quaternion, state->quaternion, attitude_error);

    for (int i = 0; i < 3; ++i) {
        double rate_error = setpoint->angular_rate[i] - state->angular_rate[i];
        rate_error *= ctrl->rate_weight;
        out_cmd->body_torque[i] =
            cs_pid_update(&ctrl->axes[i], attitude_error[i], rate_error, dt);
    }

    out_cmd->collective_thrust = 0.0;
}
