#include <math.h>
#include <stdio.h>
#include <string.h>

#include "attitude/quaternion.h"
#include "control_types.h"
#include "mixer.h"
#include "pid.h"

static void quaternion_integrate(double q[4], const double omega[3], double dt) {
    double dq[4];
    dq[0] = -0.5 * (q[1] * omega[0] + q[2] * omega[1] + q[3] * omega[2]);
    dq[1] =  0.5 * (q[0] * omega[0] + q[2] * omega[2] - q[3] * omega[1]);
    dq[2] =  0.5 * (q[0] * omega[1] - q[1] * omega[2] + q[3] * omega[0]);
    dq[3] =  0.5 * (q[0] * omega[2] + q[1] * omega[1] - q[2] * omega[0]);

    q[0] += dq[0] * dt;
    q[1] += dq[1] * dt;
    q[2] += dq[2] * dt;
    q[3] += dq[3] * dt;
    quaternion_normalize(q);
}

static void print_status(double t,
                         const cs_state_t* state,
                         const cs_attitude_setpoint_t* setpoint) {
    double err[3];
    double setpoint_q[4] = {
        setpoint->quaternion[0],
        setpoint->quaternion[1],
        setpoint->quaternion[2],
        setpoint->quaternion[3],
    };
    quaternion_normalize(setpoint_q);
    double q_curr[4] = {
        state->quaternion[0],
        state->quaternion[1],
        state->quaternion[2],
        state->quaternion[3],
    };
    quaternion_normalize(q_curr);

    double q_inv[4];
    quaternion_inverse(q_curr, q_inv);
    double q_err[4];
    quaternion_multiply(setpoint_q, q_inv, q_err);
    err[0] = 2.0 * q_err[1];
    err[1] = 2.0 * q_err[2];
    err[2] = 2.0 * q_err[3];

    printf("t=%.3f s | attitude error = [%+.4f, %+.4f, %+.4f] rad | "
           "rates = [%+.3f, %+.3f, %+.3f] rad/s\n",
           t,
           err[0], err[1], err[2],
           state->angular_rate[0],
           state->angular_rate[1],
           state->angular_rate[2]);
}

int main(void) {
    /* Basic quadrotor: arms along +Y,+X,-Y,-X. */
    cs_rotor_config_t rotors[4];
    const double arm = 0.25;
    const double thrust_coeff = 1.0;
    const double torque_coeff = 0.02;

    rotors[0] = (cs_rotor_config_t){ {0.0, arm, 0.0}, {0.0, 0.0, 1.0},  1.0, thrust_coeff, torque_coeff };
    rotors[1] = (cs_rotor_config_t){ {-arm, 0.0, 0.0}, {0.0, 0.0, 1.0}, -1.0, thrust_coeff, torque_coeff };
    rotors[2] = (cs_rotor_config_t){ {0.0, -arm, 0.0}, {0.0, 0.0, 1.0},  1.0, thrust_coeff, torque_coeff };
    rotors[3] = (cs_rotor_config_t){ {arm, 0.0, 0.0}, {0.0, 0.0, 1.0}, -1.0, thrust_coeff, torque_coeff };

    cs_mixer_t mixer;
    if (cs_mixer_init(&mixer, rotors, 4) != 0) {
        fprintf(stderr, "Mixer initialisation failed\n");
        return 1;
    }

    cs_pid_gains_t gains[3] = {
        {.kp = 2.5, .ki = 0.3, .kd = 0.15, .integrator_limit = 2.0, .output_limit = 5.0},
        {.kp = 2.5, .ki = 0.3, .kd = 0.15, .integrator_limit = 2.0, .output_limit = 5.0},
        {.kp = 1.2, .ki = 0.2, .kd = 0.10, .integrator_limit = 1.0, .output_limit = 3.0},
    };
    cs_attitude_pid_t controller;
    cs_attitude_pid_init(&controller, gains, 1.0);

    cs_state_t state = {
        .position = {0.0, 0.0, 0.0},
        .velocity = {0.0, 0.0, 0.0},
        .quaternion = {cos(15.0 * M_PI / 180.0 / 2.0), sin(15.0 * M_PI / 180.0 / 2.0), 0.0, 0.0},
        .angular_rate = {0.0, 0.0, 0.0},
    };

    cs_attitude_setpoint_t setpoint = {
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .angular_rate = {0.0, 0.0, 0.0},
    };

    const double dt = 0.002;
    const double sim_time = 2.0;
    const int steps = (int)(sim_time / dt);

    const double inertia[3] = {0.02, 0.02, 0.04};

    for (int k = 0; k < steps; ++k) {
        cs_actuator_command_t cmd;
        cs_attitude_pid_update(&controller, &setpoint, &state, dt, &cmd);
        cmd.collective_thrust = 9.81;

        double omega[CS_MAX_ROTORS] = {0.0};
        cs_mixer_mix(&mixer, &cmd, omega);

        double applied[4] = {0.0, 0.0, 0.0, 0.0};
        for (int r = 0; r < 4; ++r) {
            for (size_t i = 0; i < mixer.rotor_count; ++i) {
                applied[r] += mixer.allocation[r][i] * omega[i] * omega[i];
            }
        }

        double angular_accel[3] = {
            applied[0] / inertia[0],
            applied[1] / inertia[1],
            applied[2] / inertia[2],
        };

        for (int i = 0; i < 3; ++i) {
            state.angular_rate[i] += angular_accel[i] * dt;
        }
        quaternion_integrate(state.quaternion, state.angular_rate, dt);

        double thrust_body[3] = {0.0, 0.0, applied[3]};
        (void)thrust_body;

        double t = (k + 1) * dt;
        if (k % 50 == 0 || k == steps - 1) {
            print_status(t, &state, &setpoint);
        }
    }

    return 0;
}
