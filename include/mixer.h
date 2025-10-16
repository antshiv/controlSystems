#ifndef CONTROL_SYSTEMS_MIXER_H
#define CONTROL_SYSTEMS_MIXER_H

#include <stddef.h>

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CS_MAX_ROTORS 8

typedef struct {
    double position[3];
    double axis[3];
    double direction;
    double thrust_coeff;
    double torque_coeff;
} cs_rotor_config_t;

typedef struct {
    size_t rotor_count;
    cs_rotor_config_t rotors[CS_MAX_ROTORS];
    double allocation[4][CS_MAX_ROTORS];
    double pseudo[CS_MAX_ROTORS][4];
} cs_mixer_t;

int cs_mixer_init(cs_mixer_t* mixer,
                  const cs_rotor_config_t* rotors,
                  size_t rotor_count);

int cs_mixer_mix(const cs_mixer_t* mixer,
                 const cs_actuator_command_t* command,
                 double* omega_out);

void cs_mixer_eval(const cs_mixer_t* mixer,
                   const double* omega,
                   cs_actuator_command_t* out_command);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEMS_MIXER_H */
