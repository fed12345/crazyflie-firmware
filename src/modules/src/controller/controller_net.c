#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"
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
//static attitude_t rateDesired;
static float actuatorThrust = 0;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;
static float nn_input[1][13];
static uint32_t time;
float nn_output[1][4];

float nn_in0;
float nn_in1;
float nn_in2;
float nn_in3;
float nn_in4;
float nn_in5;
float nn_in6;
float nn_in7;
float nn_in8;
float nn_in9;
float nn_in10;
float nn_in11;
float nn_in12;
float nn_out0;
float nn_out1;
float nn_out2;
float nn_out3;

void controllerNetInit(void)
{
    DEBUG_PRINT("controllerNetInit\n");
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

    if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);

    }
    
    if (RATE_DO_EXECUTE(NET_RATE, stabilizerStep)) {
        time = start - usecTimestamp();

        nn_control(control, sensors, state);
        // This is the part I keep
        attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                nn_output[0][0], nn_output[0][1], nn_output[0][2]);

        attitudeControllerGetActuatorOutput(&control->roll,
                                            &control->pitch,
                                            &control->yaw);

        control->yaw = -control->yaw;
        control->thrust =  nn_output[0][3]*4998.66013476f;
        
        cmd_thrust = control->thrust;
        cmd_roll = control->roll;
        cmd_pitch = control->pitch;
        cmd_yaw = control->yaw;
        r_roll = radians(sensors->gyro.x);
        r_pitch = -radians(sensors->gyro.y);
        r_yaw = radians(sensors->gyro.z);
        accelz = sensors->acc.z;
    }

  //control->thrust = actuatorThrust;

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

bool deterministic = true;

const float output_std[4] = {
    0.9520828127861023,
    0.9748178124427795,
    0.9901017546653748,
    0.8914892077445984,
};

const float gate_pos[NUM_GATES][3] = {
    {2.0, -1.5, -1.5},
    {2.0, 1.5, -1.5},
    {-2.0, 1.5, -1.5},
    {-2.0, -1.5, -1.5},
    {2.0, -1.5, -1.5},
    {2.0, 1.5, -1.5},
    {-2.0, 1.5, -1.5},
    {-2.0, -1.5, -1.5},
};

const float gate_yaw[NUM_GATES] = {
    0.7853981852531433,
    2.356194496154785,
    3.9269907474517822,
    5.497786998748779,
    0.7853981852531433,
    2.356194496154785,
    3.9269907474517822,
    5.497786998748779,
};

const float start_pos[3] = {
    -2.0, -1.5, -1.5
};

const float gate_pos_rel[NUM_GATES][3] = {
    {2.8284265995025635, 2.82842755317688, 0.0},
    {2.1213202476501465, 2.1213202476501465, 0.0},
    {2.8284270763397217, 2.8284270763397217, 0.0},
    {2.1213202476501465, 2.1213204860687256, 0.0},
    {2.8284265995025635, 2.82842755317688, 0.0},
    {2.1213202476501465, 2.1213202476501465, 0.0},
    {2.8284270763397217, 2.8284270763397217, 0.0},
    {2.1213202476501465, 2.1213204860687256, 0.0},
};

const float gate_yaw_rel[NUM_GATES] = {
    -4.71238899230957,
    1.570796251296997,
    1.570796251296997,
    1.570796251296997,
    -4.71238899230957,
    1.570796251296997,
    1.570796251296997,
    1.570796251296997,
};

uint8_t target_gate_index = 0;
float pos[3];
float vel[3];

void nn_reset() {
    target_gate_index = 0;
}

float deg2rad(float deg){
    return deg * 3.1415f / 180.0f;
}

float rad2deg(float rad){
    return rad * 180.0f / 3.1415f;
}

void nn_control(control_t *control, const sensorData_t *sensors, const state_t *state) {
    // Get the current position, velocity and heading
    pos[0] = state->position.x;
    pos[1] = state->position.y;
    pos[2] = -state->position.z;

    
    vel[0] = state->velocity.x;
    vel[1] = state->velocity.y;
    vel[2] = -state->velocity.z;

    // set the position and heading of the target gate
    float target_pos[3] = {gate_pos[target_gate_index][0], gate_pos[target_gate_index][1], gate_pos[target_gate_index][2]};
    float target_yaw = gate_yaw[target_gate_index];

    // Set the target gate index to the next gate if we passed through the current one
    if (cosf(target_yaw) * (pos[0] - target_pos[0]) + sinf(target_yaw) * (pos[1] - target_pos[1]) > 0) {
        target_gate_index++;
        // loop back to the first gate if we reach the end
        target_gate_index = target_gate_index % NUM_GATES;
        // reset the target position and heading
        target_pos[0] = gate_pos[target_gate_index][0];
        target_pos[1] = gate_pos[target_gate_index][1];
        target_pos[2] = gate_pos[target_gate_index][2];
        target_yaw = gate_yaw[target_gate_index];
    }

    // Get the position of the drone in gate frame
    float pos_rel[3] = {
        cosf(target_yaw) * (pos[0] - target_pos[0]) + sinf(target_yaw) * (pos[1] - target_pos[1]),
        -sinf(target_yaw) * (pos[0] - target_pos[0]) + cosf(target_yaw) * (pos[1] - target_pos[1]),
        pos[2] - target_pos[2]
    };

    // Get the velocity of the drone in gate frame
    float vel_rel[3] = {
        cosf(target_yaw) * vel[0] + sinf(target_yaw) * vel[1],
        -sinf(target_yaw) * vel[0] + cosf(target_yaw) * vel[1],
        vel[2]
    };

    // Get the heading of the drone in gate frame
    float yaw_rel = -deg2rad(state->attitude.yaw) - target_yaw;
    while (yaw_rel > 3.1415f) {yaw_rel -= 2*3.1415f;}
    while (yaw_rel < -3.1415f) {yaw_rel += 2*3.1415f;}

    // position and velocity
    for (int i = 0; i < 3; i++) {
        nn_input[0][i] = pos_rel[i];
        nn_input[0][i+3] = vel_rel[i];
    }
    
    // attitude
    nn_input[0][6] = deg2rad(state->attitude.roll); // roll
    nn_input[0][7] = deg2rad(state->attitude.pitch); // pitch
    nn_input[0][8] = yaw_rel; // yaw
    // body rates
    nn_input[0][9] = deg2rad(sensors->gyro.x); // p
    nn_input[0][10] = deg2rad(-sensors->gyro.y); // q
    nn_input[0][11] = deg2rad(-sensors->gyro.z); // r

    nn_input[0][12] = sensors->acc.z * 9.81f; // z acceleration

    // relative gate positions and headings
    for (int i = 0; i < GATES_AHEAD; i++) {
        uint8_t index = target_gate_index + i + 1;
        // loop back to the first gate if we reach the end
        index = index % NUM_GATES;
        nn_input[0][13+4*i]   = gate_pos_rel[index][0];
        nn_input[0][13+4*i+1] = gate_pos_rel[index][1];
        nn_input[0][13+4*i+2] = gate_pos_rel[index][2];
        nn_input[0][13+4*i+3] = gate_yaw_rel[index];
    }
    // Get the neural network output and write to the action array
   
    entry(nn_input, nn_output);

    // For logging
    nn_in0 = nn_input[0][0];
    nn_in1 = nn_input[0][1];
    nn_in2 = nn_input[0][2];
    nn_in3 = nn_input[0][3];
    nn_in4 = nn_input[0][4];
    nn_in5 = nn_input[0][5];
    nn_in6 = nn_input[0][6];
    nn_in7 = nn_input[0][7];
    nn_in8 = nn_input[0][8];
    nn_in9 = nn_input[0][9];
    nn_in10 = nn_input[0][10];
    nn_in11 = nn_input[0][11];
    nn_in12 = nn_input[0][12];


    nn_out0 = nn_output[0][0];
    nn_out1 = nn_output[0][1];
    nn_out2 = nn_output[0][2];
    nn_out3 = nn_output[0][3];

    //DEBUG_PRINT("nn_output: %f, %f, %f, %f\n",(double) nn_output[0][0],(double) nn_output[0][1], (double) nn_output[0][2], (double) nn_output[0][3]);
    for (int i = 0; i < 4; i++) {
        // clip the output to the range [-1, 1]
        if (nn_output[0][i] > 1) {nn_output[0][i] = 1;}
        if (nn_output[0][i] < -1) {nn_output[0][i] = -1;}
    }
    // map the output to the correct ranges
    float p_min = -1.0;
    float p_max = 1.0;
    float q_min = -1.0;
    float q_max = 1.0;
    float r_min = -1.0;
    float r_max = 1.0;
    float T_min = 0.0;
    float T_max = 1.15*9.81;
    nn_output[0][0] = rad2deg((nn_output[0][0] + 1) / 2 * (p_max - p_min) + p_min);
    nn_output[0][1] = rad2deg((nn_output[0][1] + 1) / 2 * (q_max - q_min) + q_min);
    nn_output[0][2] = rad2deg((nn_output[0][2] + 1) / 2 * (r_max - r_min) + r_min);
    nn_output[0][3] = rad2deg((nn_output[0][3] + 1) / 2 * (T_max - T_min) + T_min);
   
}

LOG_GROUP_START(nncontrol)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, n_in0, &nn_in0)
LOG_ADD(LOG_FLOAT, n_in1, &nn_in1)
LOG_ADD(LOG_FLOAT, n_in2, &nn_in2)
LOG_ADD(LOG_FLOAT, n_in3, &nn_in3)
LOG_ADD(LOG_FLOAT, n_in4, &nn_in4)
LOG_ADD(LOG_FLOAT, n_in5, &nn_in5)
LOG_ADD(LOG_FLOAT, n_in6, &nn_in6)
LOG_ADD(LOG_FLOAT, n_in7, &nn_in7)
LOG_ADD(LOG_FLOAT, n_in8, &nn_in8)
LOG_ADD(LOG_FLOAT, n_in9, &nn_in9)
LOG_ADD(LOG_FLOAT, n_in10, &nn_in10)
LOG_ADD(LOG_FLOAT, n_in11, &nn_in11)
LOG_ADD(LOG_FLOAT, n_in12, &nn_in12)
LOG_ADD(LOG_FLOAT, n_out0, &nn_out0)
LOG_ADD(LOG_FLOAT, n_out1, &nn_out1)
LOG_ADD(LOG_FLOAT, n_out2, &nn_out2)
LOG_ADD(LOG_FLOAT, n_out3, &nn_out3)

LOG_GROUP_STOP(nncontrol)