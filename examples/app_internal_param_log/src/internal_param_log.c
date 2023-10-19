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
#define NUM_GATES 8

enum State {
  IDLE,
  WAIT_FOR_TAKEOFF,
  TAKEOFF,
  WAIT_FOR_START,
  START_POS,
  WAIT_FOR_RACE,
  RACE,
  WAIT_FOR_LANDING,
  LANDING
};






uint8_t flightmode = 0;
float posAI[3] = {0,0,0};
float posExt[3] = {0,0,0};
float posSetpoint[3] = {0,0,0};
static enum State state = IDLE;
uint8_t takeoffSwitch = 0;
void appMain()
{
  DEBUG_PRINT("This is the GateNet app...\n");

  // Log - External position
  // logVarId_t extPosxid = logGetVarId("locSrv", "x");
  // logVarId_t extPosyid = logGetVarId("locSrv", "y");
  // logVarId_t extPoszid = logGetVarId("locSrv", "z");

  paramVarId_t paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
  // Set the controller to net
  paramSetInt(paramIdStabilizerController, 1);

  while(1) {
    vTaskDelay(M2T(10));

    // Get the logging data
    if(supervisorIsTumbled()) {
        crtpCommanderHighLevelStop();
        }
    switch (state)
    {
    case IDLE:
      if(takeoffSwitch == 1){
        state = WAIT_FOR_TAKEOFF;
      }
      break;
    case WAIT_FOR_TAKEOFF:
      crtpCommanderHighLevelGoTo(0, 0, 1, 0, 5, false);
      state = TAKEOFF;
      break;
    case TAKEOFF:
       if(crtpCommanderHighLevelIsTrajectoryFinished()){
          state = WAIT_FOR_START;
        }
      break;
    case WAIT_FOR_START:
      crtpCommanderHighLevelGoTo(-2, -1.5, 1.5, 0, 5, false);
      state = START_POS;
      break;
    case START_POS:
      if(crtpCommanderHighLevelIsTrajectoryFinished()){
        state = WAIT_FOR_RACE;
        paramSetInt(paramIdStabilizerController, 5);
      }
      break;
    case WAIT_FOR_RACE:
      crtpCommanderHighLevelGoTo(2, 1.5, 1.5, 0, 20, false);
      state = RACE;
      break;
    case RACE:
      if(crtpCommanderHighLevelIsTrajectoryFinished()){
        state = WAIT_FOR_LANDING;
        paramSetInt(paramIdStabilizerController, 1);
      }
      break;
    case WAIT_FOR_LANDING:
      crtpCommanderHighLevelGoTo(0, 0, 0.5, 0, 5, false);
      state = LANDING;
      break;
    case LANDING:
      if(crtpCommanderHighLevelIsTrajectoryFinished()){
        state = IDLE;
        takeoffSwitch = 0;
      }
      break;
    default:
      break;
    }
    //DEBUG_PRINT("Estimator: %d\n", status);
    //DEBUG_PRINT("Optitrack position: %f, %f, %f\n", posx, posy, posz);
    
  }
}

PARAM_GROUP_START(app_flight)
PARAM_ADD_CORE(PARAM_UINT8, flightmode, &flightmode)
PARAM_ADD_CORE(PARAM_UINT8, takeoffSwitch, &takeoffSwitch)
PARAM_GROUP_STOP(app_flight)

