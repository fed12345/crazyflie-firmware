/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * internal_log_param_api.c - App layer application of the internal log
 *  and param api  
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "estimator_kalman.h"

#include "commander.h"
#include "crtp_commander_high_level.h"
#include "mem.h"
#include "pm.h"
#include "app_channel.h"
#include "system.h"
#include "supervisor.h"
#include "supervisor_state_machine.h"
#include "usec_time.h"

#define DEBUG_MODULE "INTERNLOGPARAM"

enum State {
  WAIT_FOR_SETPOINT,
  AT_SETPOINT,
};

struct Quaternion {
    float w, x, y, z;
};

void quaternion2rotationMatrix(struct Quaternion q, float R[3][3]) {
    R[0][0] = 1.0f - 2.0f*(q.y * q.y + q.z * q.z);
    R[0][1] = 2.0f*(q.x * q.y - q.w * q.z);
    R[0][2] = 2.0f*(q.x * q.z + q.w * q.y);
    
    R[1][0] = 2.0f*(q.x * q.y + q.w * q.z);
    R[1][1] = 1.0f - 2.0f*(q.x * q.x + q.z * q.z);
    R[1][2] = 2.0f*(q.y * q.z - q.w * q.x);
    
    R[2][0] = 2.0f*(q.x * q.z - q.w * q.y);
    R[2][1] = 2.0f*(q.y * q.z + q.w * q.x);
    R[2][2] = 1.0f - 2.0f*(q.x * q.x + q.y * q.y);
}

void transformPoint(float P_local[3], float P_global[3], struct Quaternion q, float P_setpoint_global[3]) {
    float R[3][3];
    quaternion2rotationMatrix(q, R);
    
    // Rotate the local point to the global frame
    for (int i = 0; i < 3; i++) {
        P_setpoint_global[i] = 0;
        for (int j = 0; j < 3; j++) {
            P_setpoint_global[i] += R[i][j] * P_local[j];
        }
    }
    
    // Translate to global position
    for (int i = 0; i < 3; i++) {
        P_setpoint_global[i] += P_global[i];
    }
}

static enum State state = WAIT_FOR_SETPOINT;
uint8_t flightmode = 0;
float posAI[3] = {0,0,0};
float posExt[3] = {0,0,0};
float posSetpoint[3] = {0,0,0};
struct Quaternion qdrone;
void appMain()
{
  DEBUG_PRINT("This is the GateNet app...\n");
  // Log - ai deck
  logVarId_t aiPosXid = logGetVarId("aideck", "posx");
  logVarId_t aiPosYid = logGetVarId("aideck", "posy");
  logVarId_t aiPosZid = logGetVarId("aideck", "posz");

  // Log - External position
  logVarId_t extPosxid = logGetVarId("locSrv", "x");
  logVarId_t extPosyid = logGetVarId("locSrv", "y");
  logVarId_t extPoszid = logGetVarId("locSrv", "z");

  // Log - Pose with quaternions
  logVarId_t qwid = logGetVarId("kalman", "q0");
  logVarId_t qxid = logGetVarId("kalman", "q1");
  logVarId_t qyid = logGetVarId("kalman", "q2");
  logVarId_t qzid = logGetVarId("kalman", "q3");

  logVarId_t timeID = logGetVarId("controller", "time");

  paramVarId_t idEstimator = paramGetVarId("app_flight", "flightmode");
  paramVarId_t paramIdStabilizerController = paramGetVarId("stabilizer", "controller");

  // Set the controller to net
  paramSetInt(paramIdStabilizerController, 1);

  while(1) {
    vTaskDelay(M2T(10));

    int time = logGetInt(timeID);
    DEBUG_PRINT("Time: %d\n", time);
    // Get the logging data
    flightmode = paramGetInt(idEstimator);
    if(supervisorIsTumbled()) {
        crtpCommanderHighLevelStop();
        flightmode = 0;
    }

    if(flightmode){
      // Race
      posAI[0] = logGetFloat(aiPosXid);
      posAI[1] = logGetFloat(aiPosYid);
      posAI[1] = logGetFloat(aiPosZid);

      // Allocate Quaternions
      qdrone.w = logGetFloat(qwid);
      qdrone.x = logGetFloat(qxid);
      qdrone.y = logGetFloat(qyid);
      qdrone.z = logGetFloat(qzid);

      //Allocate Extrnal Position
      posExt[0] = logGetFloat(extPosxid);
      posExt[1] = logGetFloat(extPosyid);
      posExt[2] = logGetFloat(extPoszid);

      //Transform point
      transformPoint(posAI, posExt, qdrone, posSetpoint);

      //set setpoint to gate
      switch (state)
      {
      case WAIT_FOR_SETPOINT:
        //crtpCommanderHighLevelGoTo(posSetpoint[0], posSetpoint[1], posSetpoint[2], 0, 5, false);
        crtpCommanderHighLevelGoTo(1, 0.5, 1, 0, 5, true);
        state = AT_SETPOINT;
        break;
      case AT_SETPOINT:
        if(crtpCommanderHighLevelIsTrajectoryFinished()){
          state = WAIT_FOR_SETPOINT;
        }
        break;
      default:
        break;
      }


    }
    //DEBUG_PRINT("Estimator: %d\n", status);
    //DEBUG_PRINT("Optitrack position: %f, %f, %f\n", posx, posy, posz);
    
  }
}

PARAM_GROUP_START(app_flight)
PARAM_ADD_CORE(PARAM_UINT8, flightmode, &flightmode)
PARAM_GROUP_STOP(app_flight)

LOG_GROUP_START(app_flight)
LOG_ADD(LOG_INT8, flightmode, &flightmode)
LOG_GROUP_STOP(app_flight)