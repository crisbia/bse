#include "bseScene.h"
#include "bseBody.h"
#include "bseShape.h"

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
Body* Body::create(Scene* scene, BodyDesc* desc)
{
  return new Body(scene, desc);
}

//---------------------------------------------------------------------------------------------------------------------
Body::Body(Scene* scene, const BodyDesc* bodyDesc) :
    m_scene(scene),
    m_position(0,0),
    m_orientation(0),
    m_linearVelocity(0,0),
    m_angularVelocity(0),
    m_forceAccum(0,0),
    m_torqueAccum(0),
    m_currentIsland(0),
    m_dirtyShapes(32)
{
  m_userData = 0;

  m_desc = *bodyDesc;
  m_mass = m_desc.mass;
  m_invMass = (m_mass > 0) ? 1.0f / m_mass :0.0f;
  m_inertia = m_desc.inertia;
  m_invInertia = (m_inertia > 0) ? 1.0f / m_inertia : 0.0f;

  m_static = false;
  m_flags = m_desc.flags;
  m_shapesChanged = 0;

  m_hasMoved = false;
  m_aabb = AABB(getPosition(), getPosition()); // point aabb

  // set correct mass values if static flag is set
  if (isFlagSet(BSE_BODYFLAG_STATIC))
  {
    m_static = true;
      // immovable
    m_invMass = 0.0f;
    m_invInertia = 0.0f;
  }

  setPosition(m_desc.position);
  setOrientation(m_desc.orientation);

#ifdef BSE_DEBUG_FRAMEWORK
  m_lastSolverImpulse.set(0,0);
#endif
}

//---------------------------------------------------------------------------------------------------------------------
void Body::releaseShapes()
{
  for (bse::UInt i=0; i<(bse::UInt)m_shapes.size(); ++i)
  {
    Shape* shape = m_shapes[i];
    getScene()->releaseShape(shape);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Body::destroyBody(Body* body)
{
  delete body;
}

//---------------------------------------------------------------------------------------------------------------------
Body::~Body()
{

}

//---------------------------------------------------------------------------------------------------------------------
void Body::updateVelocities(bse::Real deltaTime)
{
  if (isStatic())
    return;

  // add the gravity
  // TODO probably here I could have a list of "force fields", more general.
  if (!isFlagSet(BSE_BODYFLAG_DISABLE_GRAVITY))
    addForce(getScene()->getGravity() * getMass());

  // integrate velocities
  m_linearVelocity += (m_forceAccum * (deltaTime * m_invMass));
  m_angularVelocity += (m_torqueAccum * (deltaTime * m_invInertia));
}

//---------------------------------------------------------------------------------------------------------------------
void Body::updatePositions(bse::Real deltaTime)
{
  if (isStatic())
    return;

  // integrate position
  m_position += (m_linearVelocity * deltaTime);
  m_orientation += (m_angularVelocity * deltaTime);
  m_rotationMatrix.set(m_orientation);

  setMoved();
}

//---------------------------------------------------------------------------------------------------------------------
void Body::updateDynamics(bse::Real /*deltaTime*/)
{
}

//---------------------------------------------------------------------------------------------------------------------
void Body::applyVelocityDampings(bse::Real /*deltaTime*/)
{
  if (isStatic())
    return;

  // implicit damping... directly damp velocities, no additional forces
  m_angularVelocity *= (1-getAngularDamping());
  m_linearVelocity *= (1-getLinearDamping());
}

//---------------------------------------------------------------------------------------------------------------------
void Body::updateAABB()
{
  // Update the AABB forcing all the shapes to update theirs.
  if (getShapes().empty())
  {
    m_aabb = AABB(m_position, m_position);
    m_hasMoved = false;
    return;
  }

  ShapesList &shapes = getShapes();
  Shape* shape = shapes[0];

  AABB aabb;
  shape->updateAABB();
  shape->getAABB(aabb);

  for(size_t i=1; i<shapes.size(); ++i)
  {
    shape = shapes[i];
    shape->updateAABB();
    AABB shapeAabb;
    shape->getAABB(shapeAabb);
    aabb.enclose(shapeAabb);
  }

  m_aabb = aabb;

  m_hasMoved = false;
  m_shapesChanged = false;
  m_dirtyShapes.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Body::computeAABB(AABB& aabb)
{
  // Compute the aabb. This assumes that the shapes' aabbs are up to date.
  if (getShapes().empty())
  {
    aabb = AABB(m_position, m_position);
    return;
  }

  ShapesList &shapes = getShapes();
  Shape* shape = shapes[0];

  AABB shapeAabb;
  shape->getAABB(shapeAabb);

  for(size_t i=1; i<shapes.size(); ++i)
  {
    shape = shapes[i];
    shape->getAABB(shapeAabb);
    aabb.enclose(shapeAabb);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Body::addShape(Shape* shape)
{
  // should I search the list, if the
  m_shapes.push_back(shape);

  // before the next broadphase, the aabb will be updated
  setShapesChanged();

  updateAABB();
}

//---------------------------------------------------------------------------------------------------------------------
void Body::removeShape(Shape* shape)
{
  // to be implemented
//  m_shapes.remove(shape);
  bse::UInt numShapes = static_cast<bse::UInt>(m_shapes.size());
  for (bse::UInt shapeIndex = 0; shapeIndex<numShapes; ++shapeIndex)
  {
    Shape* currentShape = m_shapes[shapeIndex];
    if (currentShape==shape)
    {
      if (shapeIndex!=numShapes-1)
      {
        m_shapes[shapeIndex] = m_shapes[numShapes-1]; // put the last in the position of the deleted one
      }

      m_shapes.pop_back();
      break;
    }
  }

  setShapesChanged();
}

//---------------------------------------------------------------------------------------------------------------------
void Body::setMoved()
{
  m_hasMoved = true;
  m_dirtyShapes.setAll();

  // mark all its shapes as moved...
  for (ShapesList::iterator shapesIter = m_shapes.begin(); shapesIter != m_shapes.end(); ++shapesIter)
  {
    (*shapesIter)->setMoved();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Body::resetMoved()
{
  m_hasMoved = false;

  // mark all its shapes as moved...
  for (ShapesList::iterator shapesIter = m_shapes.begin(); shapesIter != m_shapes.end(); ++shapesIter)
  {
    (*shapesIter)->resetMoved();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Body::applyImpulseRel(const bse::Vec2& impulse, const bse::Vec2& point)
{
  m_linearVelocity += m_invMass * impulse;
  m_angularVelocity += m_invInertia * bseCross(point, impulse);
}

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2 Body::getLocalPointVelocity(const bse::Vec2& point) const
{
  return getLinearVelocity() + bseCross(getAngularVelocity(), bseMul(getRotationMatrix(), point));
}

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2 Body::getWorldSpacePoint(const bse::Vec2& relPoint) const
{
  return getPosition() + bseMul(getRotationMatrix(), relPoint);
}

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2 Body::getLocalSpacePoint(const bse::Vec2& point) const
{
  return bseMulT(getRotationMatrix(), point - getPosition());
}

//---------------------------------------------------------------------------------------------------------------------
const BodyInfluencesList& Body::getBodyInfluences() const
{
  return m_bodyInfluences;
}

//---------------------------------------------------------------------------------------------------------------------
void Body::setCurrentIsland(Island* island)
{
  m_currentIsland = island;
}

//---------------------------------------------------------------------------------------------------------------------
Island* Body::getCurrentIsland() const
{
  return m_currentIsland;
}

//---------------------------------------------------------------------------------------------------------------------
void Body::addBodyInfluence(Body* other, Contact* contact, BodyInfluenceType influenceType)
{
  BodyInfluence influence(other, contact, influenceType);
  m_bodyInfluences.insert(influence);
}

//---------------------------------------------------------------------------------------------------------------------
void Body::resetBodyInfluences()
{
  m_bodyInfluences.clear();
  m_currentIsland = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void Body::addForceAtPoint(const bse::Vec2& force, const bse::Vec2& point)
{
  m_forceAccum += force;
  m_torqueAccum += bseCross((point-getPosition()), force);
}

//---------------------------------------------------------------------------------------------------------------------
void Body::addForceAtRelPoint(const bse::Vec2& force, const bse::Vec2& relPoint)
{
  m_forceAccum += force;
  m_torqueAccum += bseCross(relPoint, force);
}

//---------------------------------------------------------------------------------------------------------------------
void Body::setMaterial(Material* mat)
{
  for (size_t i=0; i<m_shapes.size(); ++i)
  {
    m_shapes[i]->setMaterial(mat);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Body::setShapeClean(Shape* shape)
{
  size_t shapeIndex = (size_t)-1;
  for (size_t i=0; i<m_shapes.size(); ++i)
  {
    if (m_shapes[i] == shape)
    {
      shapeIndex = i;
    }
  }

  BSE_ASSERT(shapeIndex != (size_t)-1); // Shape not found!!
  m_dirtyShapes.resetBit(shapeIndex);
  if (m_dirtyShapes.allClear())
  {
    // All the shapes' aabbs are up to date. Safely update the aabb for the body.
    m_hasMoved = false;
    m_shapesChanged = false;
    computeAABB(m_aabb);
  }
}

}
}
