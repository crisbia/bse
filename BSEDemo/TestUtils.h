#ifndef _H_INCLUDED_TEST_UTILS
#define _H_INCLUDED_TEST_UTILS

#include "bsePhysics.h"
#include "TestFramework.h"

namespace BSEDemo
{
//---------------------------------------------------------------------------------------------------------------------
class ContactPointInfo
{
public:
  ContactPointInfo(const ContactPointInfo& source) { position = source.position; normal = source.normal; }
  ContactPointInfo() { position.x = position.y = 0; normal.x = normal.y; }
  ContactPointInfo(const bse::Vec2& pos, const bse::Vec2 &norm) { position = pos; normal = norm; }
  bse::Vec2 position;
  bse::Vec2 normal;
};

//---------------------------------------------------------------------------------------------------------------------
extern void createGround(bse::phx::Scene* scene, bse::Real angle = 0, const bse::Vec2& groundDims = bse::Vec2(15.0f, 0.5f));

//---------------------------------------------------------------------------------------------------------------------
extern void createRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY);

//---------------------------------------------------------------------------------------------------------------------
extern void createPolygonalRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY);

//---------------------------------------------------------------------------------------------------------------------
extern void createPolygon(bse::phx::PolygonDesc& polyDesc, const bse::Real extRadius, const int numEdges);

//---------------------------------------------------------------------------------------------------------------------
class ObjectState
{
public:
  bse::phx::Shape* shape;

  void save(bse::phx::Shape* shape);

  bse::Vec2 position;
  float   orientation;
  bse::Vec2 linearVelocity;
  float   angularVelocity;
};

//---------------------------------------------------------------------------------------------------------------------
class ObjectMouseForce : public TestTool
{
public:
  ObjectMouseForce();

  virtual void update(float physicsDt, float aiDt);
  virtual void render(float physicsDt, float aiDt);

  bse::phx::Shape* pick();
  void setStrength(float strength) { m_strength = strength; }
  void reset() { m_currentObject = 0; m_objectState.save(0); };
  bse::phx::Shape* getCurrentObject() const { return m_currentObject; }
protected:
  virtual void doEnable() {}
  virtual void doDisable() { m_currentObject = 0; }

  bse::Vec2   m_offset;
  bse::Vec2   m_position;
  bse::phx::Scene* m_scene;
  bse::phx::Shape* m_currentObject;
  float     m_strength;
  ObjectState m_objectState;
};

//---------------------------------------------------------------------------------------------------------------------
class ObjectSelector : public TestTool
{
public:
  ObjectSelector();

  virtual void update(float physicsDt, float aiDt);
  virtual void render(float physicsDt, float aiDt);

  bse::phx::Body* pick();
  void reset() { m_currentObject = 0; };
  bse::phx::Body* getCurrentObject() const { return m_currentObject; }

protected:
  virtual void doEnable() {}
  virtual void doDisable() { m_currentObject = 0; }

  bse::phx::Scene* m_scene;
  bse::phx::Body*  m_currentObject;
  bse::Vec2   m_position;
};

} // namespace BSEDemo


#endif // _H_INCLUDED_TEST_UTILS
