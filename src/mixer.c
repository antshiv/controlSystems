#include "mixer.h"

#include <math.h>
#include <string.h>

static double vec_norm(const double v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vec_cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static int invert_4x4(const double in[4][4], double out[4][4]) {
    double a[4][8];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            a[i][j] = in[i][j];
            a[i][j + 4] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < 4; ++i) {
        int pivot_row = i;
        double max_val = fabs(a[i][i]);
        for (int r = i + 1; r < 4; ++r) {
            double val = fabs(a[r][i]);
            if (val > max_val) {
                max_val = val;
                pivot_row = r;
            }
        }
        if (max_val < 1e-12) {
            return 0;
        }
        if (pivot_row != i) {
            for (int c = 0; c < 8; ++c) {
                double tmp = a[i][c];
                a[i][c] = a[pivot_row][c];
                a[pivot_row][c] = tmp;
            }
        }

        double pivot = a[i][i];
        for (int c = 0; c < 8; ++c) {
            a[i][c] /= pivot;
        }

        for (int r = 0; r < 4; ++r) {
            if (r == i) continue;
            double factor = a[r][i];
            for (int c = 0; c < 8; ++c) {
                a[r][c] -= factor * a[i][c];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = a[i][j + 4];
        }
    }
    return 1;
}

int cs_mixer_init(cs_mixer_t* mixer,
                  const cs_rotor_config_t* rotors,
                  size_t rotor_count) {
    if (!mixer || !rotors || rotor_count == 0 || rotor_count > CS_MAX_ROTORS) {
        return -1;
    }
    memset(mixer, 0, sizeof(*mixer));
    mixer->rotor_count = rotor_count;
    for (size_t i = 0; i < rotor_count; ++i) {
        mixer->rotors[i] = rotors[i];
    }

    for (size_t i = 0; i < rotor_count; ++i) {
        cs_rotor_config_t* rotor = &mixer->rotors[i];
        double axis_norm = vec_norm(rotor->axis);
        if (axis_norm < 1e-9) {
            return -2;
        }
        double axis_unit[3] = {
            rotor->axis[0] / axis_norm,
            rotor->axis[1] / axis_norm,
            rotor->axis[2] / axis_norm
        };
        double thrust_coeff = rotor->thrust_coeff;
        double reaction_coeff = rotor->torque_coeff * rotor->direction;

        double moment_dir[3];
        vec_cross(rotor->position, axis_unit, moment_dir);

        double torque_from_thrust[3] = {
            moment_dir[0] * thrust_coeff,
            moment_dir[1] * thrust_coeff,
            moment_dir[2] * thrust_coeff
        };

        double reaction[3] = {
            axis_unit[0] * reaction_coeff,
            axis_unit[1] * reaction_coeff,
            axis_unit[2] * reaction_coeff
        };

        mixer->allocation[0][i] = torque_from_thrust[0] + reaction[0];
        mixer->allocation[1][i] = torque_from_thrust[1] + reaction[1];
        mixer->allocation[2][i] = torque_from_thrust[2] + reaction[2];
        mixer->allocation[3][i] = thrust_coeff * axis_unit[2];
    }

    double BBT[4][4] = {{0}};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (size_t k = 0; k < rotor_count; ++k) {
                sum += mixer->allocation[r][k] * mixer->allocation[c][k];
            }
            BBT[r][c] = sum;
        }
    }

    double BBT_inv[4][4];
    if (!invert_4x4(BBT, BBT_inv)) {
        return -3;
    }

    for (size_t k = 0; k < rotor_count; ++k) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0.0;
            for (int c = 0; c < 4; ++c) {
                sum += mixer->allocation[c][k] * BBT_inv[c][r];
            }
            mixer->pseudo[k][r] = sum;
        }
    }

    return 0;
}

int cs_mixer_mix(const cs_mixer_t* mixer,
                 const cs_actuator_command_t* command,
                 double* omega_out) {
    if (!mixer || !command || !omega_out) {
        return -1;
    }
    double wrench[4] = {
        command->body_torque[0],
        command->body_torque[1],
        command->body_torque[2],
        command->collective_thrust
    };

    for (size_t i = 0; i < mixer->rotor_count; ++i) {
        double omega_sq = 0.0;
        for (int r = 0; r < 4; ++r) {
            omega_sq += mixer->pseudo[i][r] * wrench[r];
        }
        if (omega_sq < 0.0) {
            omega_sq = 0.0;
        }
        omega_out[i] = sqrt(omega_sq);
    }
    return 0;
}

void cs_mixer_eval(const cs_mixer_t* mixer,
                   const double* omega,
                   cs_actuator_command_t* out_command) {
    if (!mixer || !omega || !out_command) {
        return;
    }

    double omega_sq[CS_MAX_ROTORS];
    for (size_t i = 0; i < mixer->rotor_count; ++i) {
        omega_sq[i] = omega[i] * omega[i];
    }

    double results[4] = {0.0, 0.0, 0.0, 0.0};
    for (int r = 0; r < 4; ++r) {
        double sum = 0.0;
        for (size_t k = 0; k < mixer->rotor_count; ++k) {
            sum += mixer->allocation[r][k] * omega_sq[k];
        }
        results[r] = sum;
    }

    out_command->body_torque[0] = results[0];
    out_command->body_torque[1] = results[1];
    out_command->body_torque[2] = results[2];
    out_command->collective_thrust = results[3];
}
