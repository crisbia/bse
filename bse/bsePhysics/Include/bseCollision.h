#ifndef _BSE_COLLISION_H_INCLUDED
#define _BSE_COLLISION_H_INCLUDED

#include <map>

#include "bseCommons.h"
#include "bseTypes.h"
#include "bseMath.h"
#include "bsePools.h"
#include "bseSAP.h"
#include "bsePhysicsTypes.h"

namespace bse
{
namespace phx
{

class Scene;
class Body;
class Shape;
class Circle;
class Box;
class Polygon;
class ShapePairsPool;

//---------------------------------------------------------------------------------------------------------------------
typedef enum 
{
  INVALID_BSE_BROADPHASE_TYPE = -1,
  BSE_BROADPHASE_NAIVE,
  BSE_BROADPHASE_SWEEPANDPRUNE,
  NUM_BSE_BROADPHASE_TYPES
} BroadPhaseType;

//---------------------------------------------------------------------------------------------------------------------
// basic brick for ray casting
class Ray
{
public:
  Ray() : start(0,0), end(0,0) { length = 0; }
  Ray(const bse::Vec2& s, const bse::Vec2& e) : start(s), end(e) { length = (end-start).mag();} 
  Ray(const bse::Vec2& s, const bse::Vec2& d, const bse::Real l) : start(s)
  {
    end = s + bse::Vec2(d.x*l, d.y*l);
    length = l;
  }
  bse::Vec2 start;
  bse::Vec2 end;
  bse::Real length;
};

//---------------------------------------------------------------------------------------------------------------------
class RayCastResult
{
public:
  RayCastResult() : shape(0)
  {
  }
  bse::Vec2 normal;
  bse::Vec2 point;
  const Shape* shape; // in explicit raycast, this is redundant.
};

//---------------------------------------------------------------------------------------------------------------------
union FeaturePair
{
	struct Edges
	{
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e;
	int value;
};

//---------------------------------------------------------------------------------------------------------------------
struct ClipVertex
{
	ClipVertex() { fp.value = 0; }
	bse::Vec2 v;
	FeaturePair fp;
};

//---------------------------------------------------------------------------------------------------------------------
class Contact
{
public:
  Contact() : 
      body1(0), body2(0), shape1(0), shape2(0), contactPoint(0,0), contactNormal(0,0), 
      penetration(0), restitution(0), friction(0), local1(0,0), local2(0,0), targetVelocity(0),
      positionImpulse(0), 
      normalVelocityImpulse(0),
      tangentVelocityImpulse(0),
      normalMass(0), tangentMass(0),
      separationVelocity(0),
      filtered(false)
      {
#ifdef BSE_DEBUG_HELPERS 
        totalVelocityImpulse = bse::Vec2(0,0);
        totalPositionImpulse = bse::Vec2(0,0);
#endif
      }

  Body* body1;
  Body* body2;
  Shape* shape1;
  Shape* shape2;
  bse::Vec2 contactPoint;
  bse::Vec2 contactNormal;
  bse::Real penetration;

  bse::Real restitution;
  bse::Real friction;

  // initial contact point in local space
  bse::Vec2 local1;
  bse::Vec2 local2;

  bse::Real targetVelocity;

  // position/velocity impulse for clamping
  bse::Real positionImpulse;
  bse::Real normalVelocityImpulse;
  bse::Real tangentVelocityImpulse;

  // these values are computed at the time of contact
  bse::Real normalMass;
  bse::Real tangentMass;
  bse::Real separationVelocity;
 
  // if true, the contact has been discarded by the user callback
  bool filtered;

  // useful for feedback/debug
#ifdef BSE_DEBUG_HELPERS 
  bse::Vec2 totalVelocityImpulse; 
  bse::Vec2 totalPositionImpulse; 
#endif

	FeaturePair feature;
};

//---------------------------------------------------------------------------------------------------------------------
class CollisionData
{
public:
  Contact** contacts;
  unsigned int numContacts;
public:
  void reset() { numContacts = 0; }
};


#define BSE_MAX_NUM_CONTACTS  4196

//---------------------------------------------------------------------------------------------------------------------
class ContactsPool : public ContactsList
{
public:
  ContactsPool()
  {
    resetContactsPool();
  }

  const bse::UInt getNumberOfContacts() const { return m_firstContactAvailable; }
  Contact* getComputedContact(const bse::UInt index) { BSE_ASSERT(index<m_firstContactAvailable); return &(m_contactsPool[index]); }
  Contact* getContactFromPool() 
  { 
    Contact* contact = &(m_contactsPool[m_firstContactAvailable]);
    contact->filtered = false;
    ++m_firstContactAvailable;
    return contact;
  }
  void resetContactsPool() { m_firstContactAvailable = 0; } 
protected:
  Contact m_contactsPool[BSE_MAX_NUM_CONTACTS];
  bse::UInt m_firstContactAvailable;
};

#include <map>

#define BSE_PAIR_DISABLECOLLISION (1)

//---------------------------------------------------------------------------------------------------------------------
// pair collision setup
class CollisionPair
{
public:
  CollisionPair(const Shape* s1, const Shape* s2)
  {
    if (s1<s2)
    {
      shape1 = s1; 
      shape2 = s2;
    } 
    else
    {
      shape1 = s2; 
      shape2 = s1;
    }
  }

  const Shape* shape1;
  const Shape* shape2;

  bool operator<(const CollisionPair& pair)
  {
    if (shape1 < pair.shape1) return true;
    if (shape1 == pair.shape1) return (shape2 < pair.shape2);
    return false;
  }

};

// binary operator for pairs comparison
bool operator<(const CollisionPair& pair1, const CollisionPair& pair2);

typedef char CollisionPairMode;
typedef std::map<CollisionPair, CollisionPairMode> CollisionPairsMap;
typedef std::map<CollisionPair, CollisionPairMode>::iterator CollisionPairsMapIterator;
typedef std::pair<CollisionPair, CollisionPairMode> MapPair;


//---------------------------------------------------------------------------------------------------------------------
// store a pair of candidate shapes (after broadphase)
class CandidatePair
{
public:
  Shape* shape1;
  Shape* shape2;
};

//---------------------------------------------------------------------------------------------------------------------
class NarrowPhaseCollider
{
public:
  NarrowPhaseCollider() {}
  virtual ~NarrowPhaseCollider() {}

public:
      // shared contact pool version. be careful in multi-core
  int colliderShapeShape(const Shape* shape1, const Shape* shape2, ContactsPool* pool, CollisionData* data);
  int colliderCircleCircle(const Circle* circle1, const Circle* circle2, ContactsPool* pool, CollisionData* data);
  int colliderCircleBox(const Circle* circle, const Box* box, ContactsPool* pool, CollisionData* data);
  int colliderCirclePolygon(const Circle* circle, const Polygon* polygon, ContactsPool* pool, CollisionData* data);
  int colliderBoxBox(const Box* box1, const Box* box2, ContactsPool* pool, CollisionData* data);
  int colliderBoxPolygon(const Box* box, const Polygon* polygon, ContactsPool* pool, CollisionData* data);
  int colliderPolygonPolygon(const Polygon* polygon1, const Polygon* polygon2, ContactsPool* pool, CollisionData* data);
protected:
  // non public, this is not a primitive at the moment
  int colliderPolygonPointPolygon(const bse::Vec2& point, const Polygon* polygon, Contact* contact);


public:
  // ray casting api
  bool rayCastShape(const Shape* shape, const Ray* ray, RayCastResult* result);
  bool rayCastBox(const Box* box, const Ray* ray, RayCastResult* result);
  bool rayCastCircle(const Circle* circle, const Ray* ray, RayCastResult* result);
  bool rayCastPolygon(const Polygon* box, const Ray* ray, RayCastResult* result);
};

//---------------------------------------------------------------------------------------------------------------------
class BroadPhaseCollider
{
public:
  BroadPhaseCollider(Scene* scene) : m_scene(scene) {}
  virtual ~BroadPhaseCollider() {}
  void update(ShapesList* shapes);
  void clear();
  ShapePairsPool* getCandidatePairs() { return &m_shapePairsPool; }

  virtual void addShape(Shape* newShape) = 0;
  virtual bool removeShape(const Shape* shape) = 0;

protected:
  virtual void doUpdate(ShapesList* shapes) = 0;
  virtual void doClear() = 0;

protected:
  Scene* m_scene;
  ShapePairsPool m_shapePairsPool;
};

//---------------------------------------------------------------------------------------------------------------------
class SimpleBPCollider : public BroadPhaseCollider
{
public:
  SimpleBPCollider(Scene* scene) : BroadPhaseCollider(scene) {}
  virtual ~SimpleBPCollider();

  // The brute force collider doesn't reserve a special treatment to shapes.
  virtual void addShape(Shape* newShape) {};
  virtual bool removeShape(const Shape* shape) { return true; }

protected:
  virtual void doUpdate(ShapesList* shapes);
  virtual void doClear();
};

//---------------------------------------------------------------------------------------------------------------------
class SweepAndPruneBPCollider : public BroadPhaseCollider
{
public:
  SweepAndPruneBPCollider(Scene* scene) : 
    BroadPhaseCollider(scene),
    m_sap(scene)
    {
    }

  virtual ~SweepAndPruneBPCollider();

  virtual void addShape(Shape* newShape);
  virtual bool removeShape(const Shape* shape);

protected:
  virtual void doUpdate(ShapesList* shapes);
  virtual void doClear();
private:
  SAP m_sap;
};

}
}

#endif // _BSE_COLLISION_H_INCLUDED