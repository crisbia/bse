
#include "BSEDemo.h"
#include "ParamsTweaker.h"
#include <assert.h>

namespace BSEDemo
{

Param gDemoParams[NUM_BSEDEMO_PARAMS] =
{   // name                       // type     //value  //def  //visualization       //visulized
  { BSEPARAM_DISABLE_SIMULATION,  PARAM_BOOL, INIT_PARAM_BOOL(false), INIT_PARAM_BOOL(false),   true },
  { BSEPARAM_DRAW_AABBS,          PARAM_BOOL, INIT_PARAM_BOOL(false), INIT_PARAM_BOOL(false),   true },
  { BSEPARAM_DRAW_VELOCITIES,     PARAM_BOOL, INIT_PARAM_BOOL(false), INIT_PARAM_BOOL(false),   true },
  { BSEPARAM_DRAW_BODYFRAMES,     PARAM_BOOL, INIT_PARAM_BOOL(false), INIT_PARAM_BOOL(false),   true },
  { BSEPARAM_PHYSICS_GRAVITY,     PARAM_VEC3, INIT_PARAM_VEC3(0,0,0),    INIT_PARAM_VEC3(0,0,0),   true },
};

ParamDescription gDemoParamDescription[NUM_BSEDEMO_PARAMS] =
{   // name                       // description
  { BSEPARAM_DISABLE_SIMULATION,  "Disable Simulation"  },
  { BSEPARAM_DRAW_AABBS,          "Draw AABBs"          },
  { BSEPARAM_DRAW_VELOCITIES,     "Draw Velocities"     },
  { BSEPARAM_DRAW_BODYFRAMES,     "Draw Body Frames"    },
  { BSEPARAM_PHYSICS_GRAVITY,     "Gravity"             },
};

}
