#ifndef _H_INCLUDED_PHYSICS_TESTS
#define _H_INCLUDED_PHYSICS_TESTS

#include "TestFramework.h"

namespace bse
{
namespace phx
{
class RayCastResult;
class Body;
}
}

namespace BSEDemo
{

//---------------------------------------------------------------------------------------------------------------------
class MinimalTest : public Test
{
public:
  MinimalTest();
  virtual ~MinimalTest();

  virtual void initialize();
  virtual void shutDown();
};


//---------------------------------------------------------------------------------------------------------------------
class StackTest : public Test
{
public:
  StackTest();
  virtual ~StackTest();

  virtual void initialize();
  virtual void shutDown();
};

//---------------------------------------------------------------------------------------------------------------------
class SAPTest : public Test
{
public:
  SAPTest();
  virtual ~SAPTest();

  virtual void initialize();
  virtual void shutDown();
};

//---------------------------------------------------------------------------------------------------------------------
class RayCastTest : public Test
{
public:
  RayCastTest();
  virtual ~RayCastTest();

  virtual void initialize();
  virtual void shutDown();
  virtual void execute(float phxDt, float aiDt);
  virtual void render(float phxDt, float aiDt);

  bse::phx::RayCastResult* results;

};

//---------------------------------------------------------------------------------------------------------------------
class PyramidTest : public Test
{
public:
  PyramidTest();
  virtual ~PyramidTest();

  virtual void initialize();
  virtual void shutDown();
};

//---------------------------------------------------------------------------------------------------------------------
class PolygonsTest : public Test
{
public:
  PolygonsTest();
  virtual ~PolygonsTest();

  virtual void initialize();
  virtual void shutDown();
};

//---------------------------------------------------------------------------------------------------------------------
class StressTest : public Test
{
public:
  StressTest();
  virtual ~StressTest();

  virtual void initialize();
  virtual void shutDown();
  virtual void execute(float phxDt, float aiDt);
  virtual void render(float phxDt, float aiDt);

  // test's specific variables.
  int     m_maxNumberOfObjects;       // objects number limit.
  int     m_numberOfObjects;          // current number of objects.
  float   m_objectThrowElapsedTime;   // time elapsed from when the last object has been thrown.
  float   m_platformMoveElapsedTime;  // time elapsed from when the platform was moved last time.
  bse::Vec2 m_platformRequiredMove;
  float   m_platformRequiredRot;
  float   m_platformMovePerc;

  bse::phx::Body* m_lowerBoundBody;
  bse::phx::Body* m_leftBoundBody;
  bse::phx::Body* m_rightBoundBody;
};

//---------------------------------------------------------------------------------------------------------------------
class FrictionTest : public Test
{
public:
  FrictionTest();
  virtual ~FrictionTest();

  virtual void initialize();
  virtual void shutDown();
};

}


#endif // _H_INCLUDED_PHYSICS_TESTS
