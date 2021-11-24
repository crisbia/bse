#include "bseScene.h"
#include "bseCollision.h"
#include "bseProfilingTools.h"
#include "bseIsland.h"

#include <algorithm>

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
Scene* Scene::create(SceneDesc* desc)
{
  return new Scene(desc);
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::destroy(Scene* scene)
{
  delete scene;
}

//---------------------------------------------------------------------------------------------------------------------
Body* Scene::createBody(BodyDesc* desc)
{
  // create the body
  Body* body = Body::create(this, desc);

  // get info about shapes from the descriptor and create them
  // NB: for safety reason, the full descriptor is not stored in the body
  bse::UInt numberOfShapes = static_cast<bse::UInt>(desc->shapesDescs.size());
  for (bse::UInt shapeIndex = 0; shapeIndex<numberOfShapes; ++shapeIndex)
  {
    Shape* shape = createShape(desc->shapesDescs[shapeIndex]);
    body->addShape(shape);
    shape->setBody(body);
  }

  m_insertedBodies.push_back(body);
  return body;
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseShape(Shape* shape)
{
  // remove the shape from the scene
  bse::UInt numShapes = (bse::UInt)m_shapesList.size();
  for (bse::UInt shapeIndex = 0; shapeIndex<numShapes; ++shapeIndex)
  {
    if (m_shapesList[shapeIndex] == shape)
    {
      // first of all, update the broadphase collider.
      m_bpCollider->removeShape(shape);

      // remove it from the scene
      bse::UInt lastShapeIndex = numShapes-1;
      if (lastShapeIndex!=shapeIndex)
      {
        // put the last one here and remove from back
        m_shapesList[shapeIndex] = m_shapesList[lastShapeIndex];
      }

      // pop from back
      m_shapesList.pop_back();

      m_removedShapes.push_back(shape);
      break;
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseBody(Body* body)
{
  // take the body from the global bodies list and put it on the removed bodies list

  // remove the body from the scene
  bse::UInt numBodies = (bse::UInt)m_bodiesList.size();
  for (bse::UInt bodyIndex = 0; bodyIndex<numBodies; ++bodyIndex)
  {
    if (m_bodiesList[bodyIndex] == body)
    {
      // remove it from the scene
      bse::UInt lastBodyIndex = numBodies-1;
      if (lastBodyIndex!=bodyIndex)
      {
        // put the last one here and remove from back
        m_bodiesList[bodyIndex] = m_bodiesList[lastBodyIndex];
      }

      // pop from back
      m_bodiesList.pop_back();

      body->releaseShapes();

      m_removedBodies.push_back(body);
      break;
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::destroyBody(Body* body)
{
  if (body && body->getScene() == this)
  {
    delete body;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::storeCreatedObjects()
{
  for (BodiesList::const_iterator bodyIter = m_insertedBodies.begin(); bodyIter != m_insertedBodies.end(); ++bodyIter)
  {
    m_bodiesList.push_back((*bodyIter));
  }
  m_insertedBodies.clear();

  for (ShapesList::const_iterator shapeIter = m_insertedShapes.begin(); shapeIter != m_insertedShapes.end(); ++shapeIter)
  {
    m_shapesList.push_back((*shapeIter));
    m_bpCollider->addShape((*shapeIter));
  }
  m_insertedShapes.clear();
}

//---------------------------------------------------------------------------------------------------------------------
Shape* Scene::createShape(ShapeDesc* shapeDesc)
{
  Shape* shape = Shape::createShape(this, shapeDesc);
  m_insertedShapes.push_back(shape);

  return shape;
}

//---------------------------------------------------------------------------------------------------------------------
Circle* Scene::createCircle(CircleDesc* circleShapeDesc)
{
  Circle* shape = Shape::createCircle(this, circleShapeDesc);
  m_shapesList.push_back(shape);
  return shape;
}

//---------------------------------------------------------------------------------------------------------------------
Box* Scene::createBox(BoxDesc* boxShapeDesc)
{
  Box* shape = Shape::createBox(this, boxShapeDesc);
  m_shapesList.push_back(shape);
  return shape;
}

//---------------------------------------------------------------------------------------------------------------------
Polygon* Scene::createPolygon(PolygonDesc* polygonShapeDesc)
{
  Polygon* shape = Shape::createPolygon(this, polygonShapeDesc);
  m_shapesList.push_back(shape);
  return shape;
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::destroyShape(Shape* shape)
{
  Shape::destroyShape(shape);
}

//---------------------------------------------------------------------------------------------------------------------
Ray* Scene::createRay()
{
  Ray* ray = new Ray(); // the ray constructor is public, because it can be used flexibly outside the scene.
  m_raysList.push_back(ray);
  return ray;
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::destroyRay(Ray* ray)
{
  bool found = false;
  for (size_t i=0; i<m_raysList.size(); ++i)
  {
    if (ray == m_raysList[i])
    {
      m_raysList[i] = m_raysList.back();
      found = true;
      break;
    }
  }
  if (found)
  {
    delete ray;
    m_raysList.pop_back();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseRays()
{
  for (RaysList::iterator iter=m_raysList.begin(); iter!=m_raysList.end(); ++iter)
    delete (*iter);

  m_raysList.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseMaterials()
{
  for (MaterialsList::iterator iter=m_materials.begin(); iter != m_materials.end(); ++iter)
    delete (*iter);

  m_materials.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::clear()
{
  releaseBodies();
  releaseRays();
  releaseMaterials();
  m_bpCollider->clear();
}

//---------------------------------------------------------------------------------------------------------------------
Scene::Scene(const SceneDesc* sceneDesc) :
  m_internalClock(0),
  m_simulatedTime(0),
  m_defaultMaterial(0),
  m_contactFeedback(0),
  m_contactFilter(0)
{
  // copy the descriptor internally (just for safety, any further change will be ignored)
  m_sceneDesc = *sceneDesc;

  // store some useful data
  m_timeStep = m_sceneDesc.timeStep;
  m_solverIterations = m_sceneDesc.solverIterations;
  m_flags = m_sceneDesc.flags;
  m_gravity = m_sceneDesc.gravity;

  initializeDefaultMaterial();

  switch (m_sceneDesc.broadPhaseType)
  {
  case BSE_BROADPHASE_SWEEPANDPRUNE:
    m_bpCollider = new SweepAndPruneBPCollider(this);
    break;
  case BSE_BROADPHASE_NAIVE:
  default:
    m_bpCollider = new SimpleBPCollider(this);
  break;
  }

#ifdef BSE_ENABLE_PROFILER
  m_profiler = new bse::Profiler();
#else
  m_profiler = 0;
#endif
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseBodies()
{
  for (size_t i=0; i<m_bodiesList.size(); ++i)
  {
    m_removedBodies.push_back(m_bodiesList[i]);
  }
  m_bodiesList.clear();

  for (size_t i=0; i<m_shapesList.size(); ++i)
  {
    m_removedShapes.push_back(m_shapesList[i]);
  }
  m_shapesList.clear();
}

//---------------------------------------------------------------------------------------------------------------------
Scene::~Scene()
{
  releaseDefaultMaterial();
  releaseBodies(); // this releases shapes too
  releaseMaterials();
  releaseRays();

  cleanUpReleasedObjects();

  delete m_bpCollider;

#ifdef BSE_ENABLE_PROFILER
  delete m_profiler;
#endif
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::solveVelocityConstraints(const ContactsList &contacts)
{
  // contact solver
  if (contacts.empty())
    return;

  // iterate the contacts
  int velocityIterations = getSolverIterations();

  size_t cIndex;
  size_t numContacts = contacts.size();
  for (int iter=0; iter<velocityIterations; ++iter)
  {
    // Solve normal constraints. NB: the total normal impulse is going to be reused to solve the tangent constraints
    for (cIndex = 0; cIndex < numContacts; ++cIndex)
    {
        Contact* contact = contacts[cIndex];
        if (contact->filtered)
          continue;
        Body* body1 = contact->body1;
        Body* body2 = contact->body2;
        if (body1->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE) ||
            body2->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE)
            ) // || contact->separationVelocity>0.0f)
          continue; // skip this contact

        bse::Vec2 pos1 = bseMul(body1->getRotationMatrix(), contact->local1);
        bse::Vec2 pos2 = bseMul(body2->getRotationMatrix(), contact->local2);

        bse::Vec2 normal = contact->contactNormal;
        bse::Real targetVelocity = contact->targetVelocity;

        bse::Vec2 relPos1 = pos1;
        bse::Vec2 relPos2 = pos2;

        bse::Real relVelNorm = bseDot(
              normal,
              body1->getLinearVelocity() + bseCross(body1->getAngularVelocity(), relPos1)
            - body2->getLinearVelocity() - bseCross(body2->getAngularVelocity(), relPos2));

        bse::Vec2 impulseVector(0,0);

        bse::Vec2 temp1Norm = body1->getInvInertia() * bseCross(relPos1, normal);
        bse::Vec2 temp2Norm = body2->getInvInertia() * bseCross(relPos2, normal);

//        bse::Real rest = restitution;

        bse::Real lambda = (targetVelocity - relVelNorm) /
          (body1->getInvMass() + body2->getInvMass() + bseDot( normal, bseCross( temp1Norm, relPos1 ) + bseCross( temp2Norm, relPos2 ) ) );

        // I should accumulate and clamp...
        bse::Real newImpulse = bseMax(contact->normalVelocityImpulse + lambda, 0.0f);
        bse::Real impulseNorm = newImpulse - contact->normalVelocityImpulse;

        impulseVector = bse::Vec2(normal.x * impulseNorm, normal.y * impulseNorm);

        body1->applyImpulseRel(impulseVector, relPos1);
        body2->applyImpulseRel(-impulseVector, relPos2);

        contact->normalVelocityImpulse = newImpulse;
    }
#if 1
    // TODO this code needs optimizations and some more testing, but it's mostly working.

    // solve tangent constraints
    for (cIndex = 0; cIndex < numContacts; ++cIndex)
    {
      Contact* contact = contacts[cIndex];
      if (contact->filtered)
        continue;

      Body* body1 = contact->body1;
      Body* body2 = contact->body2;

      if (body1->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE) || body2->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE))
        continue; // skip this contact

      bse::Vec2 pos1 = bseMul(body1->getRotationMatrix(), contact->local1);
      bse::Vec2 pos2 = bseMul(body2->getRotationMatrix(), contact->local2);

      bse::Vec2 normal = contact->contactNormal;
      bse::Real friction = contact->friction;

      bse::Vec2 relPos1 = pos1;
      bse::Vec2 relPos2 = pos2;

      bse::Vec2 relVel =
        body1->getLinearVelocity() + bseCross(body1->getAngularVelocity(), relPos1) -
        (body2->getLinearVelocity() + bseCross(body2->getAngularVelocity(), relPos2));

      bse::Vec2 tangent = bseCross( normal, 1.0f );
      bse::Real relVelTang = bseDot( tangent, relVel );

      bse::Vec2 temp1Tang = body1->getInvInertia() * bseCross(relPos1, tangent);
      bse::Vec2 temp2Tang = body2->getInvInertia() * bseCross(relPos2, tangent);

      bse::Real impulseTang = -relVelTang /
        (body1->getInvMass() + body2->getInvMass() + bseDot( tangent, bseCross( temp1Tang, relPos1 ) + bseCross( temp2Tang, relPos2 ) ) );

//////////////////////////////////////////////////////////////
#if 0
      // This is an equivalent formula.
		  float rt1 = bseDot(relPos1, tangent);
		  float rt2 = bseDot(relPos2, tangent);
		  float kTangent = body1->getInvMass() + body2->getInvMass();
		  kTangent += body1->getInvInertia() * (bseDot(relPos1, relPos1) - rt1 * rt1) + body2->getInvInertia() * (bseDot(relPos2, relPos2) - rt2 * rt2);
		  float massTangent = 1.0f /  kTangent;

      bse::Real impulseTang = -relVelTang * massTangent;
#endif
 ////////////////////////////////////////////////////////////

      bse::Real maxFriction = friction * contact->normalVelocityImpulse;
      bse::Real newImpulse  = bseClamp(contact->tangentVelocityImpulse + impulseTang, -maxFriction, maxFriction);

      impulseTang = newImpulse - contact->tangentVelocityImpulse;

      bse::Vec2 impulseVector = (tangent * impulseTang);

      body1->applyImpulseRel(impulseVector, relPos1);
      body2->applyImpulseRel(-impulseVector, relPos2);

      contact->tangentVelocityImpulse = newImpulse;
    }
#endif
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::solvePositionConstraints(const ContactsList& contacts)
{
  // now solve the correct the positions (nb: we can have a different "number of iterations" value from the one used in the velocities)
  if (contacts.empty())
    return;

  int positionIterations = getSolverIterations();

  size_t contactIndex;
  size_t numContacts = contacts.size();
  for (int iterationIndex=0; iterationIndex<positionIterations; ++iterationIndex)
  {
    bse::Real minSeparation = 10000.0f;
    const bse::Real beta = 0.1f; // baumgarte correction.
    const bse::Real linearSlop = 0.0f; // 0.005f;	// 0.5 cm
      // Prevent large corrections
    const bse::Real linearCorrection = 0.1f;

    for(contactIndex = 0; contactIndex < numContacts; ++contactIndex)
    {
      Contact* contact = contacts[contactIndex];
      if (contact->filtered)
        continue;
      // resolve positions
      Body* b1 = contact->body1;
      Body* b2 = contact->body2;
      if (b1->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE) || b2->isFlagSet(BSE_BODYFLAG_DISABLE_RESPONSE))
        continue; // skip this contact

      bse::Real invMass1 = b1->m_invMass;
      bse::Real invI1 = b1->m_invInertia;
      bse::Real invMass2 = b2->m_invMass;
      bse::Real invI2 = b2->m_invInertia;
      bse::Vec2 normal = contact->contactNormal;
      bse::Vec2 tangent = bseCross(normal, 1.0f);

      // Solver normal constraints

      // what about the rotation?
//          bse::Vec2 r1 = contact->contactPoint - b1->m_position; // ???
//          bse::Vec2 r2 = contact->contactPoint - b2->m_position;
        bse::Vec2 r1 = bseMul(b1->getRotationMatrix(), contact->local1);
        bse::Vec2 r2 = bseMul(b2->getRotationMatrix(), contact->local2);

        bse::Vec2 p1 = b1->m_position + r1;
        bse::Vec2 p2 = b2->m_position + r2;
        bse::Vec2 dp =  p1 - p2;

        // Approximate the current separation. we want separation negative, but our penetration is positive
        bse::Real separation = bseDot(dp, normal) - contact->penetration;

        separation = bseMax(separation, -linearCorrection);

        // Track max constraint error.
        minSeparation = bseMin(minSeparation, separation);

        // Compute normal impulse
        bse::Real normalMass = 1.0f / (b1->m_invMass + b2->m_invMass);
        bse::Real dImpulse = normalMass * beta * bseMin(0.0f, separation + linearSlop);

        // impulse clamping... I don't want a positive impulse
        bse::Real impulseOld = contact->positionImpulse;
        contact->positionImpulse = bseMin( impulseOld + dImpulse, 0.0f);
        dImpulse = contact->positionImpulse - impulseOld; // apply only the offset.

        bse::Vec2 impulse = dImpulse * normal;

        b1->m_position -= invMass1 * impulse;
        b1->m_orientation -= invI1 * bseCross(r1, impulse);
        b1->m_rotationMatrix.set(b1->m_orientation);

        b2->m_position += invMass2 * impulse;
        b2->m_orientation += invI2 * bseCross(r2, impulse);
        b2->m_rotationMatrix.set(b2->m_orientation);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::collisionDetection()
{
  {
#ifdef BSE_ENABLE_PROFILER
    ProfileTaskWrapper broad(m_profiler, "BSE_TASKTYPE_BROADPHASE");
#endif

    // Broad phase collision detection.
    m_bpCollider->update(&m_shapesList);
  }

  ShapePairsPool* shapePairs = m_bpCollider->getCandidatePairs();

  // contact generation, from the data in the pairs pool
  {
#ifdef BSE_ENABLE_PROFILER
    ProfileTaskWrapper contGen(m_profiler, "BSE_TASKTYPE_GENERATECONTACTS");
#endif

    m_contactsPool.resetContactsPool();

    NarrowPhaseCollider collider;
    Contact* contacts[64];
    CollisionData collisionData;
    collisionData.contacts = (Contact**)&contacts;
    collisionData.reset();

      bse::UInt numCandidates = shapePairs->getNumberOfPairs();
      for (bse::UInt candidateIndex = 0; candidateIndex < numCandidates; ++candidateIndex)
      {
        const ShapesPair* pair = shapePairs->getComputedPair(candidateIndex);
        if (pair->shape1->isStatic() && pair->shape2->isStatic())
        {
          continue;
        }

        int numContacts = 0;
        {
#ifdef BSE_ENABLE_PROFILER
          ProfileTaskWrapper contGen(m_profiler, "BSE_TASKTYPE_GENERATEPAIRCONTACTS");
#endif
          numContacts = collider.colliderShapeShape(pair->shape1, pair->shape2, &m_contactsPool, &collisionData);
          for (int iContact=0; iContact<numContacts; ++iContact)
          {
            Contact* contact = collisionData.contacts[iContact];
            Body* body1 = contact->body1;
            Body* body2 = contact->body2;
            if (body1 && body2)
            {
              body1->addBodyInfluence(body2, contact, BSE_INFLUENCE_PRIMARY);
              body2->addBodyInfluence(body1, contact, BSE_INFLUENCE_SECONDARY);
            }
          }
        }
      }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::simulateSubStep(bse::Real deltaTime)
{
// prestep
  {
    // reset interactions
    for (BodiesList::iterator bodiesIter = m_bodiesList.begin(); bodiesIter != m_bodiesList.end(); ++bodiesIter)
    {
      Body* body = (*bodiesIter);
      body->resetBodyInfluences();
    }
  }

#ifdef BSE_ENABLE_PROFILER
  m_profiler->clear();
#endif

#ifdef BSE_ENABLE_PROFILER
  ProfileTaskWrapper totalSim(m_profiler, "BSE_TASKTYPE_SIMULATE");
#endif


    {
  #ifdef BSE_ENABLE_PROFILER_DYNAMICS
      ProfileTaskWrapper intVels(m_profiler, "BSE_TASKTYPE_INTEGRATEVELOCITIES");
  #endif

      //////////////////
      // integrate the velocities
      for (BodiesList::iterator bodiesIter = m_bodiesList.begin(); bodiesIter != m_bodiesList.end(); ++bodiesIter)
      {
        Body* body = (*bodiesIter);
        body->updateVelocities(deltaTime);
      }
    }

  {
#ifdef BSE_ENABLE_PROFILER
    ProfileTaskWrapper collDet(m_profiler, "BSE_TASKTYPE_COLLISIONDETECTION");
#endif

    collisionDetection();

    //// This is the user filtering phase
    if (m_contactFilter)
    {
      bse::UInt numberoOfContacts = m_contactsPool.getNumberOfContacts();
      for (bse::UInt contactIndex = 0; contactIndex < numberoOfContacts; ++contactIndex)
      {
        Contact* contact = m_contactsPool.getComputedContact(contactIndex);
        contact->filtered = m_contactFilter->applyFilterOnContact(contact);
      }
    }

  }

  {
    #ifdef BSE_ENABLE_PROFILER_DYNAMICS
      ProfileTaskWrapper solveDyn(m_profiler, "BSE_TASKTYPE_DYNAMICS");
    #endif

    m_islandsManager.computeIslands(m_bodiesList);
    const IslandsList& islands = m_islandsManager.getIslands();

    size_t numContacts = 0;
    for (size_t iIsl = 0; iIsl<islands.size(); ++iIsl)
    {
      solveVelocityConstraints(islands[iIsl]->getContacts());
      numContacts += islands[iIsl]->getContacts().size();
    }

    {
      // apply the velocities to compute the new position/orientation
      {
        size_t numBodies = m_bodiesList.size();
        for (size_t iBody=0; iBody<numBodies; ++iBody)
        {
          Body* body = m_bodiesList[iBody];
          // TODO I still need to understand where is the best place to do this...
#if 0
          body->applyVelocityDampings(deltaTime);
#endif
          body->updatePositions(deltaTime);
          body->resetAccumulators();
        }
      }
    }

    for (size_t iIsl = 0; iIsl<islands.size(); ++iIsl)
    {
      solvePositionConstraints(islands[iIsl]->getContacts());
    }
  }

  //// send the feedback to the custom feedback manager
  // for now, just report every contact, but actually the reporting system
  // the report should be filtered here. (TODO) two ways to implement this:
  // - if you want a report, you should claim it
  // - the other way around: if I don't want a report, I should claim it
  if (m_contactFeedback)
  {
    bse::UInt numberoOfContacts = m_contactsPool.getNumberOfContacts();
    for (bse::UInt contactIndex = 0; contactIndex < numberoOfContacts; ++contactIndex)
    {
      Contact* contact = m_contactsPool.getComputedContact(contactIndex);
      m_contactFeedback->onContactReport(contact);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::cleanUpReleasedObjects()
{
  for (size_t iShape=0; iShape<m_removedShapes.size(); ++iShape)
  {
    Shape::destroyShape(m_removedShapes[iShape]);
  }

  for (size_t iBody=0; iBody<m_removedBodies.size(); ++iBody)
  {
    Body::destroyBody(m_removedBodies[iBody]);
  }

  m_removedBodies.clear();
  m_removedShapes.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::simulate(bse::Real deltaTime)
{
#if 0 // TODO internal stepping management
// update the internal clock
  m_internalClock += deltaTime;

  // compute the number of required steps
  bse::Real timeToSimulate = m_internalClock - m_simulatedTime;
  bse::UInt numSteps = (bse::UInt)floorf( timeToSimulate / getInternalTimeStep());

  for (bse::UInt stepNum=0; stepNum<numSteps; stepNum++)
    simulateSubStep(getInternalTimeStep());

  // some time could be accumulated... interpolation?
  m_simulatedTime += (numSteps * getInternalTimeStep());
#endif

// insertion / removal management
  storeCreatedObjects();

  cleanUpReleasedObjects();

  simulateSubStep(deltaTime);
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::initializeDefaultMaterial()
{
  MaterialDesc materialDesc;
  // Create the material directly, don't store it in the global list.
  m_defaultMaterial = new Material(&materialDesc);
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseDefaultMaterial()
{
  delete m_defaultMaterial;
}

//---------------------------------------------------------------------------------------------------------------------
Material* Scene::createMaterial(MaterialDesc* materialDesc)
{
  Material* material = new Material(materialDesc);
  m_materials.push_back(material);
  return material;
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::releaseMaterial(Material* material)
{
  bool found = false;
  for (size_t i=0; i<m_materials.size(); ++i)
  {
    if (material==m_materials[i])
    {
      m_materials[i] = m_materials.back();
      found = true;
      break;
    }
  }

  if (found)
  {
    delete material;
    m_materials.pop_back();
  }
}

//---------------------------------------------------------------------------------------------------------------------
Shape* Scene::pointCollision(const bse::Vec2& point)
{
//#define FAST_POINT_COLLISION
#ifdef FAST_POINT_COLLISION
  // (Experimental) Fast version, use SAP information to intersect the scene.
  // TODO fix this, it works but it's not perfect.
  bseShapesList candidates;
  bool res = m_sap.intersect(point, candidates);
  if (res)
  {
    // return the first one.
    size_t numShapes = candidates.size();
    for (size_t iShape=0; iShape<numShapes; ++iShape)
    {
      Shape* shape = candidates[iShape];
      if (shape->getAABB()->contains(point) && shape->contains(point))
      {
        return shape;
      }
    }
  }
  return 0;
#else

  // Slow version, check all the shapes.
  size_t numShapes = m_shapesList.size();
  for (size_t iShape=0; iShape<numShapes; ++iShape)
  {
    Shape* shape = m_shapesList[iShape];
    if (shape->getAABB()->contains(point) && shape->contains(point))
    {
      return shape;
    }
  }
#endif

  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
bool Scene::rayCast(Ray* ray, RayCastResult* result)
{
  // TODO use SaP data to speed up this raycast on the whole scene.
  NarrowPhaseCollider collider;
  bool retVal = false;

  // brute force
  RayCastResult tempResult;
  bse::Real distance = FLT_MAX;

  bse::UInt numberOfShapes = static_cast<bse::UInt>(m_shapesList.size());
  for (bse::UInt shapeIndex=0; shapeIndex<numberOfShapes; ++shapeIndex)
  {
    Shape* shape = m_shapesList[shapeIndex];
    bool hasCast = collider.rayCastShape(shape, ray, &tempResult);
    if (hasCast)
    {
      retVal = true;
      bse::Real dist = (tempResult.point - ray->start).mag();
      if (dist<distance)
      {
        result->point = tempResult.point;
        result->normal = tempResult.normal;
        result->shape = tempResult.shape;
        distance = dist;
      }
      // To catch the closest, need to visit the whole scene.
      // TODO add a flag or a parameter, or add a special functions to cover both cases.
#if 0
      return true;
#endif
    }
  }

  return retVal;
}

//---------------------------------------------------------------------------------------------------------------------
void Scene::setupPairCollisionDetection(const Shape* shape1, const Shape* shape2, const CollisionPairMode mode)
{
  CollisionPair pair(shape1, shape2);

  CollisionPairsMapIterator iter;

  iter = m_collisionPairs.find(pair);

  if (iter != m_collisionPairs.end())
  {
    if (mode==0) // search the pair and remove it
    {
      m_collisionPairs.erase(iter);
    } else
    {
      // modify the existing one
      (*iter).second = mode;
    }

    return;
  }

  // if I'm here, I need to insert the pair in the map.
  m_collisionPairs.insert(MapPair(pair, mode));
}

//---------------------------------------------------------------------------------------------------------------------
const CollisionPairMode Scene::getPairCollisionMode(const Shape* shape1, const Shape* shape2)
{
  CollisionPair pair(shape1, shape2);
  CollisionPairsMapIterator iter = m_collisionPairs.find(pair);

  if (iter == m_collisionPairs.end())
    return 0;

  return (*iter).second;
}

}
}
