#include "PhysicsTests.h"

#include "DrawUtils.h"

#include "TestIDs.h"
#include "TestUtils.h"
#include "bseMath.h"
#include "bsePhysics.h"

namespace BSEDemo
{

//---------------------------------------------------------------------------------------------------------------------
// Class: MinimalTest
//---------------------------------------------------------------------------------------------------------------------
MinimalTest::MinimalTest() : Test(TEST_ID_MINIMAL, "Minimal")
{

}

//---------------------------------------------------------------------------------------------------------------------
MinimalTest::~MinimalTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void MinimalTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();

  scene->getDefaultMaterial()->setRestitution(0);
  scene->getDefaultMaterial()->setFriction(0);

  bse::Vec2 boxDims = bse::Vec2(0.2f, 0.2f);
  bse::Vec2 position = bse::Vec2(0.0f, boxDims.y *0.5f);

  const float SOME_OFFSET_ON_X = 0.02f; // Add some offset on the x
  const float SOME_OFFSET_ON_Y = 0.02f; // Also add some offset on the y, so that boxes don't touch each other initially.

  const int NUM_BOXES = 1;
  const int NUM_STACKS = 1;
  const bse::Real STACKS_DIST = 0.5f;

  position.y += SOME_OFFSET_ON_Y;
  for (int j=0; j<NUM_STACKS; ++j)
  {
    for (int i=0; i<NUM_BOXES; ++i)
    {
      bse::phx::BodyDesc bodyDesc;
      bodyDesc.mass = 1;
      bodyDesc.position = position;
      bodyDesc.inertia = 0.1f;
      if (i==0)
      {
       // bodyDesc.flags |= BSE_BODYFLAG_STATIC;
      }

      bse::phx::BoxDesc shapeDesc;
      shapeDesc.dims = boxDims;
      bodyDesc.shapesDescs.push_back(&shapeDesc);

      scene->createBody(&bodyDesc);
      position.y += boxDims.y;
      position.x += SOME_OFFSET_ON_X;
      position.y += SOME_OFFSET_ON_Y;
    }
    position.x += STACKS_DIST;
    position.y = boxDims.y*0.5f;
  }

  createGround(scene, 0);
}

//---------------------------------------------------------------------------------------------------------------------
void MinimalTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

//---------------------------------------------------------------------------------------------------------------------
// Class: StackTest
//---------------------------------------------------------------------------------------------------------------------
StackTest::StackTest() : Test(TEST_ID_BODIES_STACK, "Bodies stack")
{

}

//---------------------------------------------------------------------------------------------------------------------
StackTest::~StackTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void StackTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();

  scene->getDefaultMaterial()->setRestitution(0);
  scene->getDefaultMaterial()->setFriction(1);

  const int NUM_BOXES = 3;
  const int NUM_STACKS = 1;
  const bse::Real STACKS_DIST = 0.5f;
  bse::Vec2 boxDims = bse::Vec2(0.2f, 0.2f);
  bse::Vec2 position = bse::Vec2(0.0f, boxDims.y *0.5f);

  for (int j=0; j<NUM_STACKS; ++j)
  {
    for (int i=0; i<NUM_BOXES; ++i)
    {
      bse::phx::BodyDesc bodyDesc;
      bodyDesc.mass = 1;
      bodyDesc.position = position;
      bodyDesc.inertia = 0.1f;

      bse::phx::BoxDesc shapeDesc;
      shapeDesc.dims = boxDims;
      bodyDesc.shapesDescs.push_back(&shapeDesc);

      scene->createBody(&bodyDesc);

      position.y += boxDims.y;
    }
    position.x += STACKS_DIST;
    position.y = boxDims.y*0.5f;
  }

  createGround(scene, 0);
}

//---------------------------------------------------------------------------------------------------------------------
void StackTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

//---------------------------------------------------------------------------------------------------------------------
// Class: SAPTest
//---------------------------------------------------------------------------------------------------------------------
SAPTest::SAPTest() : Test(TEST_ID_BROADPHASE_SAP, "Sweep and prune stress test")
{

}

//---------------------------------------------------------------------------------------------------------------------
SAPTest::~SAPTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void SAPTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();

  bse::Real SPHERE_RADIUS = 0.15f/2.0f;
  bse::Vec2 position = bse::Vec2(0.0f, SPHERE_RADIUS);

  // Pyramid of circles.
  const int NUM_ROWS = 20; // this means... n * (n-1) / 2 boxes, the bottom line has n boxes
  for (int i=NUM_ROWS; i>0; --i)
  {
    int numBoxes = i;
    position.x = 0 + (NUM_ROWS - i - 1) * (SPHERE_RADIUS + 0.01f);
    for (int j=0; j<numBoxes; ++j)
    {
      bse::phx::BodyDesc bodyDesc;
      bodyDesc.position = position;
      bodyDesc.mass = 1;
      bodyDesc.inertia = 0.1f;

      bse::phx::CircleDesc shapeDesc;
      shapeDesc.radius = SPHERE_RADIUS;

      bodyDesc.shapesDescs.push_back(&shapeDesc);

      scene->createBody(&bodyDesc);

      position.x += (SPHERE_RADIUS*2+0.02f);
    }
    position.y += ((SPHERE_RADIUS*2)+0.015f);
  }

  createGround(scene);
}

//---------------------------------------------------------------------------------------------------------------------
void SAPTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
}

//---------------------------------------------------------------------------------------------------------------------
// Class: RayCastTest
//---------------------------------------------------------------------------------------------------------------------
RayCastTest::RayCastTest() : Test(TEST_ID_RAYCAST, "RayCast")
{

}

//---------------------------------------------------------------------------------------------------------------------
RayCastTest::~RayCastTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void RayCastTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();
  scene->setGravity(0,0);
  scene->getDefaultMaterial()->setRestitution(1);

  float timeStep = 1.0f/60.0f;

  const int NUM_OBJS = 10;
  const bse::Real ROOM_WIDTH = 3;
  const bse::Real ROOM_HEIGHT = 3;
  const bse::Real ROOM_START_X = -ROOM_WIDTH / 2.0f;
  const bse::Real ROOM_START_Y = -ROOM_HEIGHT / 2.0f;

  const bse::Int ROOM_NUM_CELLS_X = 20;
  const bse::Int ROOM_NUM_CELLS_Y = 20;

  bool occupied[ROOM_NUM_CELLS_Y][ROOM_NUM_CELLS_X];
  for (bse::Int i=0; i<ROOM_NUM_CELLS_Y; ++i)
    for (bse::Int j=0; j<ROOM_NUM_CELLS_X; ++j)
      occupied[i][j] = false;

  bse::Int numObjs = bseMin(NUM_OBJS, ROOM_NUM_CELLS_X * ROOM_NUM_CELLS_Y);

  // compute object dimension to fit in the cells
  const bse::Real OBJ_DIMX = ROOM_WIDTH / ROOM_NUM_CELLS_X;
  const bse::Real OBJ_DIMY = ROOM_HEIGHT / ROOM_NUM_CELLS_Y;

  // cell sizes
  const bse::Real CELL_DIMX = ROOM_WIDTH / ROOM_NUM_CELLS_X;
  const bse::Real CELL_DIMY = ROOM_HEIGHT / ROOM_NUM_CELLS_Y;

  for (int i=0; i<numObjs; ++i)
  {
    bse::Int px = bseRandom(1, ROOM_NUM_CELLS_X-2);
    bse::Int py = bseRandom(1, ROOM_NUM_CELLS_Y-2);

    // generate until an empty slot is found
    while (occupied[py][px])
    {
      px = bseRandom(0, ROOM_NUM_CELLS_X);
      py = bseRandom(0, ROOM_NUM_CELLS_Y);
    }

    occupied[py][px] = true; // mark as occupied

    bse::phx::BodyDesc bodyDesc;
    bodyDesc.position.x = CELL_DIMX/2.0f + ROOM_START_X + (ROOM_WIDTH / ROOM_NUM_CELLS_X) * px;
    bodyDesc.position.y = CELL_DIMY/2.0f + ROOM_START_Y + (ROOM_HEIGHT / ROOM_NUM_CELLS_Y) * py;
    bodyDesc.mass = 1;
    bodyDesc.inertia = 0.1f;

    bse::phx::CircleDesc shapeDesc;
    shapeDesc.radius = bseMin(OBJ_DIMX, OBJ_DIMY) /2.0f;

    // store the pointer to the shape descriptor. the shape is automatically created internally.
    bodyDesc.shapesDescs.push_back(&shapeDesc);
    bse::phx::Body* body = scene->createBody(&bodyDesc);
    body->setUserData((void*)(size_t)(i+1));

    // generate a random initial velocity
    bse::Vec2 v( bseRandom(-1.0f, 1.0f), bseRandom(-1.0f, 1.0f));
    v.normalize();
    v *= 1.5f;
    body->setLinearVelocity(v);

    // EASY assumption. I create a ray for each body, so they go in pair, without any additional structure
    bse::phx::Ray* ray = scene->createRay();
    ray->start = body->getPosition();
    ray->end = ray->start + bse::Vec2(v.x*timeStep, v.y*timeStep);
  }

  createRoom(scene, ROOM_START_X, ROOM_START_Y, ROOM_WIDTH, ROOM_HEIGHT);

  results = new bse::phx::RayCastResult[numObjs];
}

//---------------------------------------------------------------------------------------------------------------------
void RayCastTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();

  delete [] results;
  results = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void RayCastTest::execute(float phxDt, float aiDt)
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  float timeStep = 1.0f / 60.0f;

  bse::phx::RaysList &raysList = scene->getRays();
  bse::phx::BodiesList &bodies = scene->getBodies();

  for (int i=0; i<(int)bodies.size(); ++i)
  {
    bse::phx::Body* body = bodies[i];
    bse::UserData udata = body->getUserData();
    if (!udata)
      continue;

    int rayIndex = -1 + (int)(size_t)udata;
    bse::phx::Ray* ray = raysList[rayIndex];

    // update ray
    bse::Vec2 v = body->getLinearVelocity();
    bse::Vec2 dir = v;
    dir.normalize();
    dir *= 0.1f;

    ray->start = dir + body->getPosition();
    const int nSteps = 60; // Look forward for a second (at 60hz).

    bse::Vec2 incr = v;
    incr *= (nSteps*timeStep);
    ray->end = ray->start + incr;

      // raycast the scene
    bool hasCast = scene->rayCast(ray, &results[rayIndex]);
    if (!hasCast)
    {
      results[rayIndex].shape = 0;
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void RayCastTest::render(float phxDt, float aiDt)
{
  PhysicsDebugDraw* debugDraw = gTestManager->getDebugDraw();
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  bse::phx::RaysList &raysList = scene->getRays();

  // Draw raycast results
  Color black(0,0,0), white(1,1,1);

  for (int i=0; i<(int)raysList.size(); ++i)
  {
    bse::phx::Ray* ray = raysList[i];
    bse::phx::RayCastResult* result = &results[i];
    if (result->shape)
    {
      debugDraw->drawCircle(result->point, 0.05f, 6, black);
      const bse::Real scale = 0.25f;
      debugDraw->drawLine(result->point, result->point + bse::Vec2(result->normal.x * scale, result->normal.y * scale), black);
      debugDraw->drawLine(ray->start, result->point, white);

      debugDraw->drawCircle(ray->start, 0.05f, 6, white);
    }
    else
    {
      debugDraw->drawLine(ray->start, ray->end, white);
      debugDraw->drawCircle(ray->start, 0.05f, 6, white);
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------
// Class: PyramidTest
//---------------------------------------------------------------------------------------------------------------------
PyramidTest::PyramidTest() : Test(TEST_ID_BODIES_PYRAMID, "Pyramid")
{

}

//---------------------------------------------------------------------------------------------------------------------
PyramidTest::~PyramidTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void PyramidTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();
  scene->getDefaultMaterial()->setRestitution(0);
  scene->getDefaultMaterial()->setFriction(1);

  const int NUM_PYRAMIDS = 1;
  const int NUM_ROWS = 20; // this means... n * (n-1) / 2 boxes, the bottom line has n boxes

  const float BOX_SIZE = 0.15f;
  const float PYR_DIST = 0.5f;

  bse::Vec2 boxDims = bse::Vec2(BOX_SIZE, BOX_SIZE);
  bse::Vec2 basePosition = bse::Vec2(0.0f, 0.0f);
  for (int iPyr=0; iPyr<NUM_PYRAMIDS; ++iPyr)
  {
    bse::Vec2 position = basePosition;
    for (int i=NUM_ROWS; i>0; --i)
    {
      int numBoxes = i;

      position.x = basePosition.x + (NUM_ROWS - i - 1) * ((BOX_SIZE + 0.02f)/2.0f);
      for (int j=0; j<numBoxes; ++j)
      {
        bse::phx::BodyDesc bodyDesc;
        bodyDesc.mass = 1;
        bodyDesc.position = position;
        bodyDesc.orientation = 0;
        bodyDesc.inertia = 0.1f;

        bse::phx::BoxDesc shapeDesc;
        shapeDesc.dims.x = BOX_SIZE;
        shapeDesc.dims.y = BOX_SIZE;

        bodyDesc.shapesDescs.push_back(&shapeDesc);

        position.x += (BOX_SIZE+0.02f);

        scene->createBody(&bodyDesc);
      }
      position.y += (BOX_SIZE + 0.015f);
    }
    basePosition.x += (NUM_ROWS * (BOX_SIZE + 0.02f) + PYR_DIST);
  }
  createGround(scene);
}

//---------------------------------------------------------------------------------------------------------------------
void PyramidTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

//---------------------------------------------------------------------------------------------------------------------
// Class: PolygonsTest
//---------------------------------------------------------------------------------------------------------------------
PolygonsTest::PolygonsTest() : Test(TEST_ID_CONVEX_POLYGONS, "Convex polygons")
{

}

//---------------------------------------------------------------------------------------------------------------------
PolygonsTest::~PolygonsTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void PolygonsTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();
  scene->setGravity(0,0);
  scene->getDefaultMaterial()->setRestitution(1);

  // create a polygonal room, as in TEST_PLAYGROUND
  const bse::Real ROOM_WIDTH = 3;
  const bse::Real ROOM_HEIGHT = 3;
  const bse::Real ROOM_START_X = -ROOM_WIDTH / 2.0f;
  const bse::Real ROOM_START_Y = -ROOM_HEIGHT / 2.0f;

  createPolygonalRoom(scene, ROOM_START_X, ROOM_START_Y, ROOM_WIDTH, ROOM_HEIGHT);


  const bse::Real EXT_RADIUS = 0.1f;
  const int MIN_NUM_EDGES = 3;
  const int MAX_NUM_EDGES = 8;

  const int NUM_OBJS = 10;
  const bse::Real X_POS_INCR = EXT_RADIUS;
  const bse::Real Y_POS_INCR = EXT_RADIUS*2.0f+0.1f;

  bse::Vec2 pos(ROOM_START_X+EXT_RADIUS*2,ROOM_START_Y+EXT_RADIUS*2);

  for (int objIndex = 0; objIndex < NUM_OBJS; ++objIndex)
  {
    bse::phx::PolygonDesc polyDesc;

    int edges = bseRandom(MIN_NUM_EDGES, MAX_NUM_EDGES);
    createPolygon(polyDesc, EXT_RADIUS, edges);

    bse::phx::BodyDesc polyBodyDesc;
    polyBodyDesc.inertia = 0.1f;
    polyBodyDesc.mass = 1;
    polyBodyDesc.shapesDescs.push_back(&polyDesc);

    bse::phx::Body* polygonBody = scene->createBody(&polyBodyDesc);
    polygonBody->setPosition(pos);
    polygonBody->setOrientation(bseRandom(0.0f, bse_pi/2.0f));
    pos.x += X_POS_INCR;
    pos.y += Y_POS_INCR;

    polygonBody->setLinearVelocity(0,-1);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void PolygonsTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

//---------------------------------------------------------------------------------------------------------------------
// Class: StressTest
//---------------------------------------------------------------------------------------------------------------------
StressTest::StressTest() : Test(TEST_ID_STRESS_TEST, "StressTest")
{

}

//---------------------------------------------------------------------------------------------------------------------
StressTest::~StressTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void StressTest::initialize()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();
  scene->getDefaultMaterial()->setRestitution(0);
  scene->getDefaultMaterial()->setFriction(0);

  // Create a container for a large number of physics objects.
  float sizeX = 2;
  float sizeY = 1.5f;
  float x = -sizeX/2;
  float y = -sizeY/2;

  bse::phx::BoxDesc x_boxDesc;
  x_boxDesc.dims.x = sizeX;
  x_boxDesc.dims.y = 0.1f; // just a bit of thickness

  bse::phx::BoxDesc y_boxDesc;
  y_boxDesc.dims.x = 0.1f;
  y_boxDesc.dims.y = sizeY;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_STATIC;

  bse::phx::Body* body;

  // lower bound
  bodyDesc.shapesDescs.push_back(&x_boxDesc);
  body = scene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x + sizeX / 2.0f, y-0.05f);
  m_lowerBoundBody = body;

  // left bound
  bodyDesc.shapesDescs.push_back(&y_boxDesc);
  body = scene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x - sizeY/2 * cos(bse_pi/4), y + sizeY/2.0f * (1 - 0.5f*sin(bse_pi/4)));
  body->setOrientation(bse_pi/4);
  m_leftBoundBody = body;

  // right bound
  bodyDesc.shapesDescs.push_back(&y_boxDesc);
  body = scene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setOrientation(-bse_pi/4);
  body->setPosition(x + sizeX + sizeY/2 * cos(bse_pi/4), y + sizeY/2.0f * (1 - 0.5f*sin(bse_pi/4)));
  m_rightBoundBody = body;

  m_maxNumberOfObjects = 1000;
  m_numberOfObjects = 0;
  m_objectThrowElapsedTime = 0;
  m_platformMoveElapsedTime = 0;
  m_platformRequiredMove = 0;
  m_platformRequiredRot = 0;
  m_platformMovePerc = 1;
}

//---------------------------------------------------------------------------------------------------------------------
void StressTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

//---------------------------------------------------------------------------------------------------------------------
void StressTest::execute(float phxDt, float aiDt)
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();

  m_objectThrowElapsedTime += phxDt;
  const float throwInterval = 0.3f;
  if (m_numberOfObjects < m_maxNumberOfObjects && m_objectThrowElapsedTime > throwInterval)
  {
    m_objectThrowElapsedTime = 0;

    bse::phx::BodyDesc bodyDesc;
    bodyDesc.mass = bseRandom(0.1f, 1.0f);
    bodyDesc.inertia = 0.01f;

    // choose the type of object randomly.
    bse::phx::CircleDesc circle;
    bse::phx::BoxDesc box;
    int t = bseRandom(0,2);
    if (t==0)
    {
      circle.radius = bseRandom(0.05f, 0.075f);
      bodyDesc.shapesDescs.push_back(&circle);
    }
    else
    {
      box.dims.x = bseRandom(0.05f, 0.15f);
      box.dims.y = bseRandom(0.05f, 0.15f);
      bodyDesc.shapesDescs.push_back(&box);
    }

    bodyDesc.position = bse::Vec2(0, 1);
    bodyDesc.orientation = bseRandom(0.0f, bse_pi/4);
    bse::phx::Body* body = scene->createBody(&bodyDesc);
    const float startVel = 0.5f;
    bse::Vec2 vel(bseRandom(-1.0f, 1.0f), bseRandom(-1.0f, 0.0f));
    vel.normalize();
    vel *= startVel;
    body->setLinearVelocity(vel);

    ++m_numberOfObjects;
  }

  //// Move the container from time to time.
  const float platformMoveInterval = 2;
  const float moveIntensity = 0.25f * phxDt;
  const float rotIntensity = bse_pi/32.0f * phxDt;
  m_platformMoveElapsedTime += phxDt;

  if (m_platformMoveElapsedTime > platformMoveInterval)
  {
    m_platformMoveElapsedTime = 0;

    // choose a random movement direction.
    bse::Vec2 direction(bseRandom(-1.0f,1.0f), bseRandom(-1.0f,1.0f));
    direction.normalize();
    direction *= moveIntensity;

    float rot = bseRandom(-1.0f, 1.0f);
    rot *= rotIntensity;

    m_platformRequiredMove = direction;
    m_platformRequiredRot = rot;
    m_platformMovePerc = 0;
  }

  if (m_platformMovePerc < 1.0f)
  {
    bse::Vec2 leftOff = m_leftBoundBody->getPosition() - m_lowerBoundBody->getPosition();
    bse::Vec2 rightOff = m_rightBoundBody->getPosition() - m_lowerBoundBody->getPosition();
    bse::Mat22 rot(m_platformRequiredRot);
    leftOff = bseMul(rot, leftOff);
    rightOff = bseMul(rot, rightOff);
    bse::Mat22 centerRot = m_lowerBoundBody->getRotationMatrix();
    centerRot.rotate(m_platformRequiredRot);

    m_lowerBoundBody->setRotationMatrix(centerRot);
    bse::Mat22 leftRot = centerRot;
    leftRot.rotate(bse_pi/4);
    m_leftBoundBody->setRotationMatrix(leftRot);
    bse::Mat22 rightRot = centerRot;
    rightRot.rotate(-bse_pi/4);
    m_rightBoundBody->setRotationMatrix(rightRot);

    m_lowerBoundBody->setPosition(m_lowerBoundBody->getPosition() + m_platformRequiredMove);
    m_leftBoundBody->setPosition(m_lowerBoundBody->getPosition() + leftOff);
    m_rightBoundBody->setPosition(m_lowerBoundBody->getPosition() + rightOff);

    m_platformMovePerc += 0.05f;
  }

}

//---------------------------------------------------------------------------------------------------------------------
void StressTest::render(float phxDt, float aiDt)
{
  drawString(300, 20, Color(0,0,0), "Number of objects: %d", m_numberOfObjects);
}

//---------------------------------------------------------------------------------------------------------------------
// Class: PolygonsTest
//---------------------------------------------------------------------------------------------------------------------
FrictionTest::FrictionTest() : Test(TEST_ID_FRICTION, "Friction")
{

}

//---------------------------------------------------------------------------------------------------------------------
FrictionTest::~FrictionTest()
{

}

//---------------------------------------------------------------------------------------------------------------------
void FrictionTest::initialize()
{
  // Drop a simple box on an oriented ground.

  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  gTestManager->saveSceneSettings();

  scene->getDefaultMaterial()->setRestitution(0);
  scene->getDefaultMaterial()->setFriction(1);

  const bse::Real ori = 3.14f / 12.0f;
  const bse::Real cOri = cos(ori);
  const bse::Real sOri = sin(ori);

  const int numBoxes = 3;
  const bse::Real s = 0.5f;
  const bse::Real dL = s * 2.0f - 0.3f;
  bse::Real friction = 1;
  const bse::Real frictionDecay = 0.2f;
  bse::Vec2 pos(dL*0.5f*numBoxes*cOri, dL*0.5f*numBoxes*sOri);
  for (int i=0; i<numBoxes; ++i)
  {
    bse::phx::MaterialDesc matDesc;
    matDesc.restitution = 0;
    matDesc.friction = friction;
    matDesc.frictionMode = bse::phx::BSE_FRICTIONMODE_MULT;
    bse::phx::Material* mat = scene->createMaterial(&matDesc);

    bse::phx::BoxDesc boxDesc;
    boxDesc.dims = bse::Vec2(s, s);

    bse::phx::BodyDesc bodyDesc;
    bodyDesc.mass = 0.1f;
    bodyDesc.inertia = 0.1f;
    bodyDesc.orientation = ori;
    bodyDesc.position = pos;
    bodyDesc.position.y += 0.5f;
    bodyDesc.position.x += 0.5f;
    bodyDesc.shapesDescs.push_back(&boxDesc);

    bse::phx::Body* body = scene->createBody(&bodyDesc);
    body->setMaterial(mat);

    pos.x -= dL * cOri;
    pos.y -= dL * sOri;
    friction *= frictionDecay;
  }

  createGround(scene, ori);
}

//---------------------------------------------------------------------------------------------------------------------
void FrictionTest::shutDown()
{
  bse::phx::Scene* scene = gTestManager->getPhysicsScene();
  scene->clear();
  gTestManager->restoreSceneSettings();
}

} // namespace BSEDemo
