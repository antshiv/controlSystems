#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

int main(void) {
    cs_pid_gains_t gains[3] = {
        {.kp = 2.0, .ki = 0.0, .kd = 0.0, .integrator_limit = 10.0, .output_limit = 10.0},
        {.kp = 2.0, .ki = 0.0, .kd = 0.0, .integrator_limit = 10.0, .output_limit = 10.0},
        {.kp = 1.0, .ki = 0.0, .kd = 0.5, .integrator_limit = 10.0, .output_limit = 10.0},
    };

    cs_attitude_pid_t controller;
    cs_attitude_pid_init(&controller, gains, 1.0);

    cs_state_t state = {
        .position = {0.0, 0.0, 0.0},
        .velocity = {0.0, 0.0, 0.0},
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .angular_rate = {0.01, -0.02, 0.1},
    };

    const double theta = 10.0 * M_PI / 180.0;
    cs_attitude_setpoint_t setpoint = {
        .quaternion = {cos(theta / 2.0), sin(theta / 2.0), 0.0, 0.0},
        .angular_rate = {0.0, 0.0, 0.0},
    };

    cs_actuator_command_t cmd;
    cs_attitude_pid_update(&controller, &setpoint, &state, 0.01, &cmd);

    const double expected_roll_torque = gains[0].kp * 2.0 * sin(theta / 2.0);
    assert(nearly_equal(cmd.body_torque[0], expected_roll_torque, 1e-3));
    assert(nearly_equal(cmd.body_torque[1], 0.0, 1e-6));

    double expected_yaw = gains[2].kd * (-state.angular_rate[2]);
    assert(nearly_equal(cmd.body_torque[2], expected_yaw, 1e-6));

    printf("PASS: attitude PID basic test\n");
    return 0;
}
