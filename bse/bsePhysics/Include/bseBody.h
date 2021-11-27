#ifndef _BSE_BODY_INCLUDED
#define _BSE_BODY_INCLUDED

#include "bseTypes.h"
#include "bseAABB.h"
#include "bseIsland.h"
#include "bseBitField.h"

namespace bse
{
namespace phx
{

typedef unsigned int bseBodyFlags;

#define BSE_BODYFLAG_DISABLE_GRAVITY    1
#define BSE_BODYFLAG_DISABLE_COLLISION  2
#define BSE_BODYFLAG_STATIC             4   
#define BSE_BODYFLAG_KINEMATIC          8   // it should invalidate the static flag   
#define BSE_BODYFLAG_DISABLE_RESPONSE   16

//---------------------------------------------------------------------------------------------------------------------
class BodyDesc
{
public:
  BodyDesc() :
    flags(0), 
    mass(1), 
    inertia(0.1f),
    position(0,0), 
    orientation(0), 
    linearDamping(0),
    angularDamping(0)
  { 
  }

  bseBodyFlags  flags;
  bse::Real mass;
  bse::Real inertia;

  // initial position, orientation
  bse::Vec2 position;
  bse::Real orientation;

  bse::Real linearDamping;
  bse::Real angularDamping;

  ShapeDescsList shapesDescs; // shapes descriptors
};

//---------------------------------------------------------------------------------------------------------------------
class Body
{
  friend class Scene;
  friend class Shape; // this allows shapes creation
public:
  static Body* create(Scene* scene, const BodyDesc& desc);
  static void destroyBody(Body* body);
  Scene* getScene() const { return m_scene; }

  void setPosition(const bse::Real& x, const bse::Real& y)
  { 
    setPosition(bse::Vec2(x,y));
  }

  void setPosition(const bse::Vec2& pos) 
  { 
    m_position = pos; 
    setMoved();
  }

  void setRotationMatrix(const Mat22& mat)
  {
    m_rotationMatrix = mat;
    m_orientation = mat.toAngle();
  }
  
  void setOrientation(const bse::Real& angle) 
  { 
    m_orientation = angle;
    m_rotationMatrix.set(angle);
 
    setMoved();
  }

  void setShapeClean(Shape* shape);

  bse::Vec2 getPosition() const { return m_position; }
  bse::Real getOrientation() const { return m_orientation; }
  Mat22 getRotationMatrix() const { return m_rotationMatrix; }

  void setLinearVelocity(const bse::Real& vx, const bse::Real& vy) { setLinearVelocity(bse::Vec2(vx,vy)); }
  void setLinearVelocity(const bse::Vec2& vel) { m_linearVelocity = vel; }
  void setAngularVelocity(const bse::Real &vel) { m_angularVelocity = vel; }
  bse::Vec2 getLinearVelocity() const { return m_linearVelocity; }
  bse::Real getAngularVelocity() const { return m_angularVelocity; }

  bse::Vec2 getLocalPointVelocity(const bse::Vec2& point) const;

  bse::Vec2 getWorldSpacePoint(const bse::Vec2& relPoint) const;
  bse::Vec2 getLocalSpacePoint(const bse::Vec2& point) const;

  bse::Real getMass() const { return m_mass; } 
  bse::Real getInvMass() const { return m_invMass; }

  bse::Real getInertia() const { return m_inertia; }
  bse::Real getInvInertia() const { return m_invInertia; }
  
  const AABB& getAABB() const { return m_aabb; }
  void getAABB(AABB& aabb) { aabb = m_aabb; }

  bool hasMoved() const { return m_hasMoved; }
  void setMoved();
  void resetMoved();

  void setShapesChanged() { m_shapesChanged = true; }
  void resetShapesChanged() { m_shapesChanged = false; }
  bool hasShapesChanged() const { return m_shapesChanged; }

  void addForce(const bse::Vec2& force) { m_forceAccum += force; }
  void addForceAtPoint(const bse::Vec2& force, const bse::Vec2& point);
  void addForceAtRelPoint(const bse::Vec2& force, const bse::Vec2& relPoint);
  
  void addTorque(const bse::Real& torque) { m_torqueAccum += torque; }

  void applyImpulseRel(const bse::Vec2& impulse, const bse::Vec2& point);

  bool isFlagSet(const bseBodyFlags& flag) { return (m_flags & flag)>0; }
  const bool isStatic() const { return m_static; }
  const bool isKinematic() const { return m_kinematic; }

  ////////////////// damping
  void setAngularDamping(const bse::Real damping) { m_desc.angularDamping = damping; }
  void setLinearDamping(const bse::Real damping)  { m_desc.linearDamping  = damping; }
  bse::Real getAngularDamping() const { return m_desc.angularDamping; }
  bse::Real getLinearDamping() const { return m_desc.linearDamping; }

  void setMaterial(Material* mat);
protected:
  Body(Scene* scene, const BodyDesc& bodyDesc);
  virtual ~Body();

  void updateAABB();

  void addShape(Shape* shape);
  void removeShape(const Shape* shape);

  bool m_shapesChanged;
  bool m_hasMoved;

protected:
  BodyDesc m_desc;
  Scene* m_scene;

  AABB m_aabb;

  bseBodyFlags m_flags;

  bool    m_static;
  bool    m_kinematic;

  bse::Real m_mass;
  bse::Real m_invMass;
  bse::Real m_inertia;
  bse::Real m_invInertia;

  bse::Vec2 m_position;
  bse::Real m_orientation;
  Mat22 m_rotationMatrix;         // rotation matrix correspondent to m_orientation angle

  bse::Vec2 m_linearVelocity;
  bse::Real m_angularVelocity; // 1 float value is enough

protected:
  bse::Vec2 m_forceAccum;
  bse::Real m_torqueAccum;

protected:
  void resetAccumulators() { resetForceAccumulator(); resetTorqueAccumulator(); }
  void resetForceAccumulator() { m_forceAccum = 0; }
  void resetTorqueAccumulator() { m_torqueAccum = 0; }
  void updateDynamics(bse::Real deltaTime);
  void updateVelocities(bse::Real deltaTime);
  void updatePositions(bse::Real deltaTime);

  void applyVelocityDampings(bse::Real deltaTime);
protected:
  ShapesList m_shapes;

////////////
public:
  ShapesList &getShapes() { return m_shapes; }

//// useful data
public:
  bse::Vec2 getLastSolverImpulse() const { return m_lastSolverImpulse; }
protected:
  bse::Vec2 m_lastSolverImpulse;

//// userdata management 
public:
  bse::UserData getUserData() const { return m_userData; }
  void setUserData(bse::UserData userData) { m_userData = userData; }
protected:
  bse::UserData m_userData;

//// connections
public:
  const BodyInfluencesList& getBodyInfluences() const;
  void setCurrentIsland(Island* island);
  Island* getCurrentIsland() const;
  void addBodyInfluence(Body* other, Contact* contact, BodyInfluenceType influenceType);
  void resetBodyInfluences();
protected:
  BodyInfluencesList m_bodyInfluences;
  Island* m_currentIsland;
private:
  void releaseShapes();

  BitField m_dirtyShapes;

  void computeAABB(AABB& aabb);
};

}
}

#endif // _BSE_BODY_INCLUDED