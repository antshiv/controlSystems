#ifndef CONTROL_SYSTEMS_PID_H
#define CONTROL_SYSTEMS_PID_H

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double kp;
    double ki;
    double kd;
    double integrator_limit; /* absolute limit on integral term */
    double output_limit;     /* absolute limit on output */
} cs_pid_gains_t;

typedef struct {
    double integrator;
    double prev_error;
    int    has_prev;
} cs_pid_state_t;

typedef struct {
    cs_pid_gains_t gains;
    cs_pid_state_t state;
} cs_pid_controller_t;

typedef struct {
    cs_pid_controller_t axes[3]; /* roll, pitch, yaw */
    double rate_weight;          /* blend between attitude error and rate error */
} cs_attitude_pid_t;

void cs_pid_init(cs_pid_controller_t* pid, const cs_pid_gains_t* gains);
void cs_pid_reset(cs_pid_controller_t* pid);
double cs_pid_update(cs_pid_controller_t* pid, double error, double error_rate, double dt);

void cs_attitude_pid_init(cs_attitude_pid_t* ctrl,
                          const cs_pid_gains_t gains[3],
                          double rate_weight);

void cs_attitude_pid_reset(cs_attitude_pid_t* ctrl);

void cs_attitude_pid_update(cs_attitude_pid_t* ctrl,
                            const cs_attitude_setpoint_t* setpoint,
                            const cs_state_t* state,
                            double dt,
                            cs_actuator_command_t* out_cmd);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEMS_PID_H */
