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
static uint32_t time;
static float world_state[13];
static float indi_cmd[4];

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
        world_state[0] = state->position.x;
        world_state[1] = -state->position.y;
        world_state[2] = -state->position.z;
        world_state[3] = state->velocity.x;
        world_state[4] = -state->velocity.y;
        world_state[5] = -state->velocity.z;
        world_state[6] = radians(state->attitude.roll);
        world_state[7] = radians(state->attitude.pitch);
        world_state[8] = -radians(state->attitude.yaw);
        world_state[9] = radians(sensors->gyro.x);
        world_state[10] = -radians(sensors->gyro.y);
        world_state[11] = -radians(sensors->gyro.z);
        world_state[12] = sensors->acc.z*9.81f;
        nn_control(world_state, indi_cmd);
        // This is the part I keep
        attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                degrees(indi_cmd[0]), degrees(indi_cmd[1]), -degrees(indi_cmd[2]));

        attitudeControllerGetActuatorOutput(&control->roll,
                                            &control->pitch,
                                            &control->yaw);

        control->yaw = -control->yaw;
        control->thrust =  indi_cmd[3]*4711.54490755f;
        
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

void nn_control(const float world_state[13], float indi_cmd[4]) {
    // Get the current position, velocity and heading
    float pos[3] = {world_state[0], world_state[1], world_state[2]};
    float vel[3] = {world_state[3], world_state[4], world_state[5]};
    float yaw = world_state[8];

    // Get the position and heading of the target gate
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
    float yaw_rel = yaw - target_yaw;
    while (yaw_rel > 3.14f) {yaw_rel -= 2*3.14f;}
    while (yaw_rel < -3.14f) {yaw_rel += 2*3.14f;}

    // Get the neural network input
    float nn_input[13+4*GATES_AHEAD];
    // position and velocity
    for (int i = 0; i < 3; i++) {
        nn_input[i] = pos_rel[i];
        nn_input[i+3] = vel_rel[i];
    }
    // attitude
    nn_input[6] = world_state[6];
    nn_input[7] = world_state[7];
    nn_input[8] = yaw_rel;
    // body rates
    nn_input[9] = world_state[9];
    nn_input[10] = world_state[10];
    nn_input[11] = world_state[11];
    // normalized thrust 
    float T_min = 0.8*9.81;
    float T_max = 1.08*9.81;
    nn_input[12] = (world_state[12] - T_min) / (T_max - T_min) * 2 - 1;

    // relative gate positions and headings
    for (int i = 0; i < GATES_AHEAD; i++) {
        uint8_t index = target_gate_index + i + 1;
        // loop back to the first gate if we reach the end
        index = index % NUM_GATES;
        nn_input[13+4*i]   = gate_pos_rel[index][0];
        nn_input[13+4*i+1] = gate_pos_rel[index][1];
        nn_input[13+4*i+2] = gate_pos_rel[index][2];
        nn_input[13+4*i+3] = gate_yaw_rel[index];
    }
    // Get the neural network output and write to the action array
    float nn_output[1][4];
    float net_input[1][13];
    for (int i = 0; i < 13; i++) {
        net_input[0][i] = nn_input[i];
    }

    entry(net_input, nn_output);
    nn_out0 = nn_output[0][0];
    nn_out1 = nn_output[0][1];
    nn_out2 = nn_output[0][2];
    nn_out3 = nn_output[0][3];
    for (int i = 0; i < 4; i++) {
        // clip the output to the range [-1, 1]
        if (nn_output[0][i] > 1) {nn_output[0][i] = 1;}
        if (nn_output[0][i] < -1) {nn_output[0][i] = -1;}
    }
    // map the output to the correct ranges
    float p_min = -0.2;
    float p_max = 0.2;
    float q_min = -0.2;
    float q_max = 0.2;
    float r_min = -0.6;
    float r_max = 0.6;
    indi_cmd[0] = (nn_output[0][0] + 1) / 2 * (p_max - p_min) + p_min;
    indi_cmd[1] = (nn_output[0][1] + 1) / 2 * (q_max - q_min) + q_min;
    indi_cmd[2] = (nn_output[0][2] + 1) / 2 * (r_max - r_min) + r_min;
    indi_cmd[3] = (nn_output[0][3] + 1) / 2 * (T_max - T_min) + T_min;
}

LOG_GROUP_START(nncontrol)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, n_in0, &world_state[0])
LOG_ADD(LOG_FLOAT, n_in1, &world_state[1])
LOG_ADD(LOG_FLOAT, n_in2, &world_state[2])  
LOG_ADD(LOG_FLOAT, n_in3, &world_state[3])
LOG_ADD(LOG_FLOAT, n_in4, &world_state[4])
LOG_ADD(LOG_FLOAT, n_in5, &world_state[5])
LOG_ADD(LOG_FLOAT, n_in6, &world_state[6])
LOG_ADD(LOG_FLOAT, n_in7, &world_state[7])
LOG_ADD(LOG_FLOAT, n_in8, &world_state[8])
LOG_ADD(LOG_FLOAT, n_in9, &world_state[9])
LOG_ADD(LOG_FLOAT, n_in10, &world_state[10])
LOG_ADD(LOG_FLOAT, n_in11, &world_state[11])
LOG_ADD(LOG_FLOAT, n_in12, &world_state[12])
LOG_ADD(LOG_FLOAT, n_out0, &indi_cmd[0])
LOG_ADD(LOG_FLOAT, n_out1, &indi_cmd[1])
LOG_ADD(LOG_FLOAT, n_out2, &indi_cmd[2])
LOG_ADD(LOG_FLOAT, n_out3, &indi_cmd[3])


LOG_GROUP_STOP(nncontrol)