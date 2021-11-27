#ifndef _BSE_SCENE_H_INCLUDED
#define _BSE_SCENE_H_INCLUDED

#include <list>
#include "bseBody.h"
#include "bseShape.h"
#include "bseMaterial.h"
#include "bseCollision.h"
#include "bsePhysicsTypes.h"
#include "bsePhysicsContext.h"
#include "bseIsland.h"

namespace bse
{
class Profiler;

namespace phx
{

//#define BSE_MAX_NUMBER_OF_RIGID_BODIES  1024
//#define BSE_MAX_NUMBER_OF_GEOMETRICS_SHAPES 2048

class Body;
class Shape;
class Ray;
class RayCastResult;
class bseProfiler;

typedef bse::UInt bseSceneFlags;

//---------------------------------------------------------------------------------------------------------------------
class SceneDesc
{
public:
  SceneDesc() { reset(); }
  virtual ~SceneDesc() { }

  // reset to default
  void reset() 
  {
    timeStep = 1.0f/ 60.0f;
    solverIterations = 10;

    flags = 0;
    gravity = 0;

    broadPhaseType = BSE_BROADPHASE_NAIVE;
  }

  bse::Real timeStep;
  bse::UInt solverIterations;
  bseSceneFlags flags;

  // friction mode
  FrictionMode frictionMode;

  // restitution mode
  RestitutionMode restitutionMode;

  // dynamics properties
  bse::Vec2 gravity;

  // broadphase
  BroadPhaseType broadPhaseType;
};

//---------------------------------------------------------------------------------------------------------------------
class Scene
{
  friend class Body;

public:
  // only this two accessors
  static Scene* create(SceneDesc* desc);
  static void destroy(Scene* scene);

  // body creation/destruction
  Body* createBody(const BodyDesc& bodyDesc);
  void releaseBody(Body* body);

private:
  void destroyBody(Body* body);

  // shape creation/destruction
  Shape* createShape(ShapeDesc* shapeDesc);
  Circle* createCircle(CircleDesc* circleShapeDesc);
  Box* createBox(BoxDesc* boxShapeDesc);
  Polygon* createPolygon(PolygonDesc* polygonShapeDesc);

  void releaseShape(Shape* shape);

  void destroyShape(Shape* shape);

public:
  Ray* createRay();
  void destroyRay(Ray* ray);

  // Raycasting: the internal collision structures are used to improve
  // the raycasting performance. SAP? other stuff?
    // return only the closest raycasted object. later some more complicated behaviours could be useful
  bool rayCast(Ray* ray, RayCastResult* result);

  Shape* pointCollision(const bse::Vec2& point);
public:
  // simulation
  void setInternalTimeStep(bse::Real timeStep) { m_timeStep = timeStep; }
  bse::Real getInternalTimeStep() const { return m_timeStep; }

  void setSolverIterations(bse::UInt numIterations) { m_solverIterations = numIterations; }
  bse::UInt getSolverIterations() const { return m_solverIterations; }

  void setGravity(const bse::Real& gx, const bse::Real& gy) { setGravity(bse::Vec2(gx, gy)); }
  void setGravity(const bse::Vec2& gravity) { m_gravity = gravity; }
  bse::Vec2 getGravity() const { return m_gravity; }

  void simulate(bse::Real deltaTime);

// internal
protected:
  // simulation of substeps
  void simulateSubStep(bse::Real deltaTime);
  
  // perform broadphase and narrowphase (contact generation) collision detection.
  void collisionDetection();

  // solve velocity constraints
  void solveVelocityConstraints(const ContactsList& contacts);
  // solve position constraints
  void solvePositionConstraints(const ContactsList& contacts);
protected:
  // no public construction/destruction allowed
  Scene(const SceneDesc* sceneDesc);
  virtual ~Scene();
protected:
  SceneDesc m_sceneDesc;

  BodiesList m_bodiesList;
  ShapesList m_shapesList;

  RaysList   m_raysList;

  unsigned int m_numberOfBodies;
  unsigned int m_numberOfShapes;

protected:
  bse::UInt m_solverIterations;
  bse::Real m_timeStep;
  bse::Real m_internalClock; // store the total time
  bse::Real m_simulatedTime; // store the actual simulated time
  bse::UInt m_flags;

  bse::Vec2 m_gravity;

///// useful accessors
public:
  BodiesList &getBodies() { return m_bodiesList; }
  ShapesList &getShapes() { return m_shapesList; }
  RaysList   &getRays()   { return m_raysList; }

protected:
  ContactsPool m_contactsPool;

  BroadPhaseCollider* m_bpCollider;

/////// materials
public:
  Material* createMaterial(MaterialDesc* materialDesc);
  void releaseMaterial(Material* material);
  Material* getDefaultMaterial() const { return m_defaultMaterial; }
protected:
  void initializeDefaultMaterial();
  void releaseDefaultMaterial();
  
  Material*     m_defaultMaterial;
  MaterialsList m_materials;

//////// Profiler. if disabled, still return a pointer, but NULL
  bse::Profiler* m_profiler;
public:
  bse::Profiler* getProfiler() const { return m_profiler; }

//// custom collision feedback
public:
  void setContactFeedback(ContactFeedback* feedback) { m_contactFeedback = feedback; }
  ContactFeedback* getContactFeedback() const { return m_contactFeedback; }
protected:
  ContactFeedback* m_contactFeedback;

//// custom contact filtering
public:
  void setContactFilter(ContactFilter* filter) { m_contactFilter = filter; }
  ContactFilter* getContactFilter() const { return m_contactFilter; }
protected:
  ContactFilter* m_contactFilter;

public:
  void setupPairCollisionDetection(const Shape* shape1, const Shape* shape2, const CollisionPairMode mode);
  const CollisionPairMode getPairCollisionMode(const Shape* shape1, const Shape* shape2);
protected:
  CollisionPairsMap m_collisionPairs;

protected:
  IslandsManager  m_islandsManager;

// insertion / removal management
protected:
  BodiesList m_insertedBodies;
  BodiesList m_removedBodies;
  ShapesList m_insertedShapes;
  ShapesList m_removedShapes;

  void storeCreatedObjects();
  void cleanUpReleasedObjects();
  void releaseBodies();
  
  void releaseRays();
  void releaseMaterials();
public:
  void clear();
};

}
}

#endif // _BSE_SCENE_H_INCLUDED
