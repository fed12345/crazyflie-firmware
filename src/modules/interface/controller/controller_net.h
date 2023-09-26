#ifndef __CONTROLLER_NET_H__
#define __CONTROLLER_NET_H__

#include "stabilizer_types.h"

void controllerNetInit(void);
bool controllerNetTest(void);
void controllerNet(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_NET_H__
