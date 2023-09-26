#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_net.h"
#include "usec_time.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "model.c"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust = 0;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;
static float network_input[1][17];
static float network_output[1][4];
static uint32_t time;

void controllerNetInit(void)
{
    attitudeControllerResetAllPID();
    positionControllerResetAllPID();
    usecTimerInit();
}

bool controllerNetTest(void)
{

  return true;
}

void controllerNet(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
    control->controlMode = controlModeLegacy;
    uint64_t start = usecTimestamp();
    // if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        network_input[0][0] = setpoint->position.x;
        network_input[0][1] = setpoint->position.y;
        network_input[0][2] = setpoint->position.z;
        network_input[0][3] = setpoint->velocity.x;
        network_input[0][4] = setpoint->velocity.y;
        network_input[0][5] = setpoint->velocity.z;
        network_input[0][6] = state->attitude.roll;
        network_input[0][7] = state->attitude.pitch;
        network_input[0][8] = state->attitude.yaw;
        network_input[0][9] = sensors->gyro.x; //rollrate
        network_input[0][10] = -sensors->gyro.y; //pitchrate
        network_input[0][11] = sensors->gyro.z; //yawrate
        network_input[0][12] = sensors->acc.z;
        network_input[0][13] = control->roll;
        network_input[0][14] = control->pitch;
        network_input[0][15] = control->yaw;
        network_input[0][16] = control->thrust;
        
        entry(network_input, network_output);
        time = start - usecTimestamp();
        // This is the part I keep
        attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                network_output[0][0], network_output[0][2], network_output[0][3]);

        attitudeControllerGetActuatorOutput(&control->roll,
                                            &control->pitch,
                                            &control->yaw);

        control->yaw = -control->yaw;
        control->thrust = network_output[0][4];
        
        cmd_thrust = control->thrust;
        cmd_roll = control->roll;
        cmd_pitch = control->pitch;
        cmd_yaw = control->yaw;
        r_roll = radians(sensors->gyro.x);
        r_pitch = -radians(sensors->gyro.y);
        r_yaw = radians(sensors->gyro.z);
        accelz = sensors->acc.z;
    //}
  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)

/**
 * @brief Time
 */
LOG_ADD(LOG_UINT32, time, &time)

LOG_GROUP_STOP(controller)
