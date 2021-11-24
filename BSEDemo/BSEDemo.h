#ifndef _BSEDEMO_H_INCLUDED
#define _BSEDEMO_H_INCLUDED

#include "ParamsTweaker.h"

/*
 * This class collect many useful data/accessor
 * to create e mantain a bse demo
 * Some of the features:
 * - Pause/Unpause the game
 * - Enable/Disable collision detection
 * - Enable/Disable gravity
 * - Enable/Disable friction
 * -
 *
 *
 *
 * The main idea is to create a Gui to manipulate all these features
 */

typedef enum {
  INVALID_BSEDEMO_PARAM = -1,
  BSEPARAM_DISABLE_SIMULATION,
  BSEPARAM_DRAW_AABBS,
  BSEPARAM_DRAW_VELOCITIES,
  BSEPARAM_DRAW_BODYFRAMES,
  BSEPARAM_PHYSICS_GRAVITY,
  NUM_BSEDEMO_PARAMS
} BSEDemoParamName;


#endif // _BSEDEMO_H_INCLUDED
