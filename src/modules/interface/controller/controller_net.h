#ifndef __CONTROLLER_NET_H__
#define __CONTROLLER_NET_H__

#include "stabilizer_types.h"

#define GATES_AHEAD 0
#define NUM_GATES 8

void controllerNetInit(void);
bool controllerNetTest(void);
void controllerNet(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

const float gate_pos[NUM_GATES][3];
const float gate_yaw[NUM_GATES];
const float start_pos[3];
uint8_t target_gate_index;

void nn_reset(void);
void nn_control(control_t *control,const sensorData_t *sensors, const state_t *state);

float deg2rad(float deg);
float rad2deg(float rad);

#endif //__CONTROLLER_NET_H__
