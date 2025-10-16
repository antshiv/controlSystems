#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mixer.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

static void build_plus_quad(cs_rotor_config_t rotors[4]) {
    const double arm = 0.25;
    const double thrust_coeff = 1.0;
    const double torque_coeff = 0.05;

    rotors[0] = (cs_rotor_config_t){
        .position = {0.0, arm, 0.0},
        .axis = {0.0, 0.0, 1.0},
        .direction = 1.0,
        .thrust_coeff = thrust_coeff,
        .torque_coeff = torque_coeff,
    };
    rotors[1] = (cs_rotor_config_t){
        .position = {-arm, 0.0, 0.0},
        .axis = {0.0, 0.0, 1.0},
        .direction = -1.0,
        .thrust_coeff = thrust_coeff,
        .torque_coeff = torque_coeff,
    };
    rotors[2] = (cs_rotor_config_t){
        .position = {0.0, -arm, 0.0},
        .axis = {0.0, 0.0, 1.0},
        .direction = 1.0,
        .thrust_coeff = thrust_coeff,
        .torque_coeff = torque_coeff,
    };
    rotors[3] = (cs_rotor_config_t){
        .position = {arm, 0.0, 0.0},
        .axis = {0.0, 0.0, 1.0},
        .direction = -1.0,
        .thrust_coeff = thrust_coeff,
        .torque_coeff = torque_coeff,
    };
}

static void verify_match(const cs_mixer_t* mixer,
                         const cs_actuator_command_t* requested,
                         const double* omega) {
    cs_actuator_command_t actual;
    cs_mixer_eval(mixer, omega, &actual);

    assert(nearly_equal(actual.collective_thrust,
                        requested->collective_thrust,
                        1e-6));

    for (int i = 0; i < 3; ++i) {
        assert(nearly_equal(actual.body_torque[i],
                            requested->body_torque[i],
                            1e-6));
    }
}

int main(void) {
    cs_rotor_config_t rotors[4];
    build_plus_quad(rotors);

    cs_mixer_t mixer;
    int rc = cs_mixer_init(&mixer, rotors, 4);
    assert(rc == 0);

    cs_actuator_command_t cmd = {
        .body_torque = {0.0, 0.0, 0.0},
        .collective_thrust = 4.0,
    };

    double omega[CS_MAX_ROTORS] = {0};
    rc = cs_mixer_mix(&mixer, &cmd, omega);
    assert(rc == 0);

    for (int i = 0; i < 4; ++i) {
        assert(nearly_equal(omega[i], 1.0, 1e-9));
    }
    verify_match(&mixer, &cmd, omega);

    cs_actuator_command_t roll_cmd;
    memset(&roll_cmd, 0, sizeof(roll_cmd));
    roll_cmd.body_torque[0] = 0.5;
    roll_cmd.collective_thrust = 4.0;

    rc = cs_mixer_mix(&mixer, &roll_cmd, omega);
    assert(rc == 0);
    verify_match(&mixer, &roll_cmd, omega);

    cs_actuator_command_t yaw_cmd;
    memset(&yaw_cmd, 0, sizeof(yaw_cmd));
    yaw_cmd.body_torque[2] = 0.2;
    yaw_cmd.collective_thrust = 4.0;

    rc = cs_mixer_mix(&mixer, &yaw_cmd, omega);
    assert(rc == 0);
    verify_match(&mixer, &yaw_cmd, omega);

    printf("PASS: mixer basic tests\n");
    return 0;
}
