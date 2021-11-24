#ifndef _H_INCLUDED_TESTIDS
#define _H_INCLUDED_TESTIDS

namespace BSEDemo
{
  typedef enum
  {
    TEST_ID_MINIMAL,
    TEST_ID_BODIES_STACK,
    TEST_ID_BROADPHASE_SAP,
    TEST_ID_RAYCAST,
    TEST_ID_BODIES_PYRAMID,
    TEST_ID_CONVEX_POLYGONS,
    TEST_ID_STRESS_TEST,
    TEST_ID_FRICTION,

    NUM_TEST_IDS
  } 
  TestIDs;

  typedef enum
  {
    TEST_TOOL_ID_OBJECT_PICKER,

    NUM_TEST_TOOL_IDS
  } 
  TestToolIDs;
}

#endif // _H_INCLUDED_TESTIDS