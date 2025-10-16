#ifndef CONTROL_SYSTEMS_CONTROL_TYPES_H
#define CONTROL_SYSTEMS_CONTROL_TYPES_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Core vehicle state consumed by controllers.
 * Mirrors dm_state_t to avoid a direct dependency on dynamic_models.
 */
typedef struct {
    double position[3];
    double velocity[3];
    double quaternion[4];   /* body-to-inertial */
    double angular_rate[3]; /* body frame rad/s */
} cs_state_t;

/**
 * Desired attitude targets expressed as quaternion + angular rates.
 */
typedef struct {
    double quaternion[4];    /* desired body-to-inertial */
    double angular_rate[3];  /* desired body rates (rad/s) */
} cs_attitude_setpoint_t;

/**
 * High-level thrust request (e.g., from altitude or position loop).
 */
typedef struct {
    double collective_thrust; /* Newtons */
} cs_thrust_setpoint_t;

/**
 * Controller output expressed as body torque plus collective thrust.
 * A mixer downstream can translate this into rotor commands.
 */
typedef struct {
    double body_torque[3];     /* Nm in body frame */
    double collective_thrust;  /* N */
} cs_actuator_command_t;

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEMS_CONTROL_TYPES_H */
