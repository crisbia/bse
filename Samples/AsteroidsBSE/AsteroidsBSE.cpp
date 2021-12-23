//////////////////////////////////////////////////////////////////////////////
//
//              AsteroidsBSE Demo - SeeBeex soft 2008-2011
//
//  This simple Asteroids clone is part of the bse framework.
//  The demo shows the following features:
//  - Simple rigid bodies dynamics
//  - Collision filtering
//  - Real-time scene modification (body/shapes creation and destruction)
//
///////////////////////////////////////////////////////////////////////////////


// system includes
#include <vector>

#include "AsteroidsBSE.h"

// demo includes
#include "DrawUtils.h"

// bse includes
#include "bsePhysics.h"
#include "bseShape.h"

#include <stack>

PhysicsDebugDraw gDebugDraw;

void fillRegularPolygonDesc(bse::phx::PolygonDesc& polyDesc, const bse::Real extRadius, const int numEdges)
{
  for (int i=0; i<numEdges; ++i)
  {
    bse::Real angle = i*2.0f*bse_pi/(bse::Real)numEdges;
    bse::Vec2 vert(extRadius*cos(angle), extRadius*sin(angle));
    polyDesc.vertList.push_back(vert);
  }
}

// settings
const bse::Real TIME_STEP = 1.0f / 60.0f;

// simple graphics container
RenderScene* gRenderScene = 0;

// main simulation container
bse::phx::Scene*   gScene = 0;

GameObj::GameObj(GameObjType t) : type(t), m_body(0)
{
}

GameObj::~GameObj()
{
  if (m_body)
    gScene->releaseBody(m_body);
}

bse::phx::AABB GameObj::getBounds() const
{
  // compute bound based on the union of shapes bound
  bse::phx::ShapesList &shapes = m_body->getShapes();
  if (shapes.size()==0)
    return bse::phx::AABB();

  bse::phx::Shape* shape = shapes[0];
  bse::phx::AABB aabb;
  shape->getAABB(aabb);
  for(size_t i=1; i<shapes.size(); ++i)
  {
    shape = shapes[i];
    aabb.enclose(shape->getAABB());
  }

  return aabb;
}

void GameObj::setPose(const bse::Vec2& position, const bse::Real orientation)
{
  m_body->setPosition(position);
  m_body->setOrientation(orientation);
}

void GameObj::getPose(bse::Vec2& position, bse::Real& orientation) const
{
  position = m_body->getPosition();
  orientation = m_body->getOrientation();
}

void GameObj::setVelocities(const bse::Vec2& velocity, const bse::Real rotation)
{
  m_body->setLinearVelocity(velocity);
  m_body->setAngularVelocity(rotation);
}

void GameObj::getVelocities(bse::Vec2& velocity, bse::Real rotation) const
{
  velocity = m_body->getLinearVelocity();
  rotation = m_body->getAngularVelocity();
}

void GameObj::registerUserData()
{
  m_body->setUserData(this);
}

void GameObj::disableCollisionDetection(const GameObj* obj) const
{
  BSE_ASSERT(m_body);
  BSE_ASSERT(obj->m_body);
  BSE_ASSERT(obj->m_body->getScene()==m_body->getScene());
  if (getShape() != obj->getShape())
    m_body->getScene()->setupPairCollisionDetection(getShape(),obj->getShape(), BSE_PAIR_DISABLECOLLISION);
}

Ship::Ship(ShipSetup* setup) : 
      GameObj(GAMEOBJ_SHIP),
      m_powerOn(false)
{
  m_setup = *setup;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_DISABLE_RESPONSE;
  bodyDesc.inertia = 0.1f;
  bodyDesc.mass = 1.0f;

  bse::Real size = m_setup.size;

  // a simple triangle it's ok for collision
  bse::phx::PolygonDesc polyDesc;
  polyDesc.vertList.push_back(bse::Vec2(0.0f,size));
  polyDesc.vertList.push_back(bse::Vec2(-size/2.0f,-size/2.0f));
  polyDesc.vertList.push_back(bse::Vec2(size/2.0f,-size/2.0f));

  bodyDesc.shapesDescs.push_back(&polyDesc);

  m_body = gScene->createBody(bodyDesc);

  bse::phx::ShapesList &shapes = m_body->getShapes();
  m_shape = shapes[0];

  setPose(m_setup.initialPosition, m_setup.initialOrientation);
  setVelocities(m_setup.initialVelocity, m_setup.initialAngVelocity);
  
  registerUserData();

  m_shootTimer.startTimer();
}

Ship::~Ship()
{
}

void Ship::drawEngine() const
{
  if (!m_powerOn) return;

  bse::Vec2 pos = m_body->getPosition();
  bse::Mat22 ori = m_body->getRotationMatrix();
  
  VertexList engineTri;
  bse::Real size = m_setup.size/3.0f;
  engineTri.push_back(bse::Vec2(-size/2.0f, 0));
  engineTri.push_back(bse::Vec2(0, -size/2.0f));
  engineTri.push_back(bse::Vec2( size/2.0f, 0));

  bse::Vec2 triPos = pos - ori.col2 * (m_setup.size/2.0f+0.001f);
  gDebugDraw.drawPolygon(triPos, ori, engineTri, Color(1,0,0), true);
}

void Ship::applyEnginePower(const bool powerOn)
{
  m_powerOn = powerOn;

  if (!powerOn) 
    return;
  
  bse::Mat22 ori = m_body->getRotationMatrix();
  bse::Vec2 force = ori.col2;
  force *= m_setup.enginePower;
  
  m_body->addForce(force);
}

void Ship::applyRotation(const int rotate)
{
  if (rotate==0)
    return;

  bse::Real ori = m_body->getOrientation();
  ori += rotate*m_setup.steeringVelocity;
  m_body->setOrientation(ori);
}

Bullet::Bullet(BulletSetup* setup) : GameObj(GAMEOBJ_BULLET)
{
  m_setup = *setup;

  bse::phx::CircleDesc sphereDesc;
  sphereDesc.radius = m_setup.size / 2.0f;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_DISABLE_RESPONSE;
  bodyDesc.mass = 0.1f;
  bodyDesc.inertia = 0.01f;
  bodyDesc.shapesDescs.push_back(&sphereDesc);

  m_body = gScene->createBody(bodyDesc);
  bse::phx::ShapesList &shapes = m_body->getShapes();
  m_shape = shapes[0];

  m_body->setPosition(m_setup.position);
  m_body->setLinearVelocity(m_setup.direction * m_setup.initialSpeed);

  registerUserData();
}

Bullet::~Bullet()
{
}

Asteroid::Asteroid(AsteroidDesc* desc) : GameObj(GAMEOBJ_ASTEROID)
{
  m_desc = *desc;

  bse::phx::PolygonDesc polyDesc;
  
  // fill the polygon
  fillRegularPolygonDesc(polyDesc, m_desc.externalRadius, m_desc.numberOfEdges);

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.inertia = 0.1f;
  bodyDesc.mass = m_desc.mass;
  bodyDesc.shapesDescs.push_back(&polyDesc);

  m_body = gScene->createBody(bodyDesc);
  bse::phx::ShapesList &shapes = m_body->getShapes();
  m_shape = shapes[0];
  setPose(m_desc.position, m_desc.orientation);
  setVelocities(m_desc.velocity, m_desc.rotation);

  registerUserData();   
}

Asteroid::~Asteroid()
{

}

void Asteroid::saveStatusToDesc(AsteroidDesc& desc)
{
  desc = m_desc;

  // fill with the current pose/velocities, useful feature.
  getPose(desc.position, desc.orientation);
  getVelocities(desc.velocity, desc.rotation);
}



AsteroidsGame::AsteroidsGame(AsteroidsGameSetup* gameSetup) : 
      m_levelReady(false), 
      m_ship(0),
      m_engineOn(false),
      m_rotate(0),
      m_shoot(false)
{
  m_gameSetup = *gameSetup;
}

AsteroidsGame::~AsteroidsGame()
{
  delete m_ship;
  emptyBulletsList();
  emptyAsteroidsList();
}

void AsteroidsGame::initializeLevel(AsteroidsGameSetup& gameSetup)
{
  if (m_levelReady)
  {
    // clear the level, then init again
  }

  m_ship = new Ship(&gameSetup.ship);

  AsteroidDesc asteroid;
  bse::Real angle = 0;
  for (int astIndex=0; astIndex<gameSetup.asteroids.numAsteroids; ++astIndex)
  {
    angle = astIndex * (2.0f*bse_pi) / (bse::Real)gameSetup.asteroids.numAsteroids; 
    asteroid.externalRadius = gameSetup.asteroids.maxRadius;
    asteroid.numberOfEdges = gameSetup.asteroids.maxEdges;
    asteroid.position = bse::Vec2(cos(angle)*gameSetup.spaceRadius, sin(angle)*gameSetup.spaceRadius);
    asteroid.orientation = 0.0f;
    asteroid.velocity = -asteroid.position;
    asteroid.velocity.normalize();
    asteroid.velocity *= 0.25f;
    asteroid.rotation = bse_pi / 4.0f;
    asteroid.mass = gameSetup.asteroids.mass;
    asteroid.numberOfPieces = 3;

    insertAsteroid(new Asteroid(&asteroid));
  }

  m_levelReady = true;
}

bool AsteroidsGame::collisionBulletAsteroid(GameObj* obj1, GameObj* obj2, Bullet* &bullet, Asteroid* &asteroid)
{
  if (obj1->type == GAMEOBJ_BULLET && obj2->type == GAMEOBJ_ASTEROID)
  {
    bullet = static_cast<Bullet*>(obj1);
    asteroid = static_cast<Asteroid*>(obj2);
    return true;
  }
  if (obj2->type == GAMEOBJ_BULLET && obj1->type == GAMEOBJ_ASTEROID)
  {
    bullet = static_cast<Bullet*>(obj2);
    asteroid = static_cast<Asteroid*>(obj1);
    return true;
  }
  return false;
}

bool AsteroidsGame::collisionShipAsteroid(GameObj* obj1, GameObj* obj2, Ship* &ship, Asteroid* &asteroid)
{
  if (obj1->type == GAMEOBJ_SHIP && obj2->type == GAMEOBJ_ASTEROID)
  {
    ship = static_cast<Ship*>(obj1);
    asteroid = static_cast<Asteroid*>(obj2);
    return true;
  }
  if (obj2->type == GAMEOBJ_SHIP && obj1->type == GAMEOBJ_ASTEROID)
  {
    ship = static_cast<Ship*>(obj2);
    asteroid = static_cast<Asteroid*>(obj1);
    return true;
  }
  return false;
}
  
void AsteroidsGame::updateGameStatus()
{
  // reset collision events
  m_collisionEvents.resize(0);

  gScene->simulate(m_gameSetup.simulationTimeStep);

  // process collision events
  std::list<CollisionPair>::iterator collisionsIter;
  Bullet* bullet;
  Ship* ship;
  Asteroid* asteroid;
  for (collisionsIter = m_collisionEvents.begin(); collisionsIter != m_collisionEvents.end(); ++collisionsIter)
  {
    GameObj* obj1 = (*collisionsIter).obj1;
    GameObj* obj2 = (*collisionsIter).obj2;

    if (collisionBulletAsteroid(obj1, obj2, bullet, asteroid))
    {
      // destroy the asteroid, dividing it in pieces. if its size is small, destroy it
      AsteroidDesc astDesc;
      asteroid->saveStatusToDesc(astDesc);
      int numberOfPieces = astDesc.numberOfPieces;

      if (numberOfPieces > 1)
      {
        float newRadius = 2*astDesc.externalRadius / numberOfPieces;
        if (newRadius > 0.1f)
        {
          float newMass = astDesc.mass / numberOfPieces;
          float halfWay = astDesc.externalRadius;
          const float ENERGY_INCREMENT = 0.5f * astDesc.velocity.mag();

          float deltaAngle = 2*bse_pi / (float)numberOfPieces;

          for (int astIndex=0; astIndex<numberOfPieces; ++astIndex)
          {
            AsteroidDesc desc;
            // create the broken pieces: TBD: nice behaviour would be, breaking the polygon along the impact
            desc.numberOfEdges = astDesc.numberOfEdges;
            desc.numberOfPieces = astDesc.numberOfPieces == 2 ? 0 : astDesc.numberOfPieces;
            
            desc.mass = newMass;
            desc.externalRadius = newRadius;

            // pose the pieces around the original one
            desc.position = astDesc.position + bse::Vec2(halfWay*cos(deltaAngle*astIndex), halfWay*sin(deltaAngle*astIndex) );
            desc.orientation = 0.0f;

            // explosion: add some energy to the system
            // n pieces, each of mass M / n, total velocity + added velocity  
            desc.velocity = desc.position - astDesc.position;
            desc.velocity.normalize();
            desc.velocity *= (ENERGY_INCREMENT / (float)numberOfPieces);
            desc.velocity += astDesc.velocity;
            desc.rotation = 0.0f;

            insertAsteroid(new Asteroid(&desc));
          }
        }

        // remove the original one
        removeAsteroid(asteroid);
        delete asteroid;  

      }       

      // destroy the bullet
      removeBullet(bullet);
      delete bullet;
      continue;
    }

    if (collisionShipAsteroid(obj1, obj2, ship, asteroid))
    {
      
      continue;
    }
  }



  if (m_ship)
  {
    m_ship->applyEnginePower(m_engineOn);
    m_ship->applyRotation(m_rotate);
    if (m_shoot)
    {
      insertBullet(m_ship->shootBullet());
    }
  }

  //      multi dimensional space management

  // bullets: bullets outside the space, are destroyed

  bse::Vec2 lower, higher;
  gRenderScene->getSceneBounds(lower, higher);
  bse::phx::AABB bounds(lower, higher);
  {
    std::stack<Bullet*> toBeRemoved;
    BulletsList::iterator bullets = m_bullets.begin();
    for ( ; bullets != m_bullets.end(); ++bullets )
    {
      // check if it's inside the space
      Bullet* bull = (*bullets);
      bse::Vec2 pos;
      bse::Real ori;
      bull->getPose(pos, ori);
      if (! bounds.contains(pos) )
        toBeRemoved.push(bull);
    }

    while (toBeRemoved.size()>0)
    {
      Bullet* bull = toBeRemoved.top();
      removeBullet(bull);
      delete bull;
      toBeRemoved.pop();
    }
  }

  {
    // asteroids: enter opposite side, same position, same velocity
    AsteroidsList::iterator asteroids = m_asteroids.begin();
    for ( ; asteroids != m_asteroids.end(); ++asteroids)
    {
      Asteroid* asteroid = (*asteroids);

      // is bbox outside scene area? in the check, remember minimum distance: the exit side.
      bse::phx::AABB aabb = asteroid->getBounds();

      // let the asteroids re-enter from the opposite side
    }
  }
}

void AsteroidsGame::engine(const bool on)
{
  m_engineOn = on;
}

void AsteroidsGame::rotate(const int rot)
{
  m_rotate = rot;
}

void AsteroidsGame::shoot(const bool sh)
{
  m_shoot = sh;
}
  
void AsteroidsGame::resetShip()
{

}

void AsteroidsGame::emptyBulletsList()
{

}

bool AsteroidsGame::removeBullet(Bullet* bullet)
{
  return m_bullets.removeFromList(bullet);
}

void AsteroidsGame::insertBullet(Bullet* bullet)
{
  if (bullet)
    m_bullets.insertInList(bullet);
}

void AsteroidsGame::emptyAsteroidsList()
{
  for (AsteroidsList::iterator iter = m_asteroids.begin(); iter != m_asteroids.end(); ++iter)
  {
    delete (*iter);
    (*iter) = 0;
  }
}
  
bool AsteroidsGame::removeAsteroid(Asteroid* asteroid)
{
  return m_asteroids.removeFromList(asteroid);
}

void AsteroidsGame::insertAsteroid(Asteroid* asteroid)
{
  m_asteroids.insertInList(asteroid);

  //// disable the collision detection between this asteroid and all the others in the scene
  AsteroidsListIter iter = m_asteroids.begin();
  for ( ; iter != m_asteroids.end(); ++iter)
  {
    Asteroid* currentAst = (*iter);
    asteroid->disableCollisionDetection(currentAst);
  }
}

bool AsteroidsGame::applyFilterOnContact(bse::phx::Contact* contact)
{
  bse::UserData uData1 = contact->body1->getUserData();
  bse::UserData uData2 = contact->body2->getUserData();

  if (uData1==0 || uData2==0) 
    return false;
 
  // return a true value for contact that we want to be discarded
  GameObj* obj1 = static_cast<GameObj*>(uData1);
  GameObj* obj2 = static_cast<GameObj*>(uData2);
  if (obj1->type == GAMEOBJ_ASTEROID && obj2->type == GAMEOBJ_ASTEROID)
    return true;

  return false;
}

void AsteroidsGame::onContactReport(const bse::phx::Contact* contact)
{
  bse::UserData uData1 = contact->body1->getUserData();
  bse::UserData uData2 = contact->body2->getUserData();

  if (uData1==0 || uData2==0) 
  return;

  GameObj* obj1 = static_cast<GameObj*>(uData1);
  GameObj* obj2 = static_cast<GameObj*>(uData2);
  if (obj1->type > obj2->type)
  {
    // I want them in order, for comparisons
    GameObj* temp = obj1;
    obj1 = obj2;
    obj2 = temp;
  }

  // I don't want duplications, so I need to check if this pair is already in list
  std::list<CollisionPair>::iterator collisionsIter;
  for (collisionsIter = m_collisionEvents.begin(); collisionsIter != m_collisionEvents.end(); ++collisionsIter)
  {
    if ( (*collisionsIter).obj1->type != obj1->type )
      continue;
    if ( (*collisionsIter).obj2->type == obj2->type )
      return; // found out the pair, skip this contact report
  }   

  // if I'm here, I need to register the pair
  CollisionPair pair;
  pair.obj1 = obj1;
  pair.obj2 = obj2;
  m_collisionEvents.push_back(pair);
}

Bullet* Ship::shootBullet()
{
  float timeFromLastShot = m_shootTimer.getElapsedTime();
  if (timeFromLastShot >= m_setup.bulletLag)
  {
    bse::Mat22 ori = m_body->getRotationMatrix();
    BulletSetup setup = m_setup.bullet;
    setup.direction = ori.col2;
    setup.position = m_body->getPosition() + bseMul( ori, bse::Vec2(0,m_setup.size + setup.size) );
    Bullet* bullet = new Bullet(&setup);

    m_shootTimer.stopTimer();
    m_shootTimer.startTimer();
    return bullet;
  }

  return 0;
}


AsteroidsGameSetup gGameSetup;
AsteroidsGame* gAsteroidsGame = 0;

void keyboardCallback(int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
  {
    switch (key)
    {
    // Shoot
    case GLFW_KEY_SPACE:
      gAsteroidsGame->shoot(true);
      break;
    // Rotate Left
    case GLFW_KEY_LEFT:
      gAsteroidsGame->rotate(1);
      break;

    // Rotate Right
    case GLFW_KEY_RIGHT:
      gAsteroidsGame->rotate(-1);
      break;

    // Accelerate forward
    case GLFW_KEY_UP:
      gAsteroidsGame->engine(true);
      break;

    case GLFW_KEY_DOWN:
      break;
    }
  }

  if (action == GLFW_RELEASE)
  {
    switch (key)
    {
      case GLFW_KEY_SPACE:
        gAsteroidsGame->shoot(false);
        break;
        // Rotate Left
      case GLFW_KEY_LEFT:
        gAsteroidsGame->rotate(0);
        break;

        // Rotate Right
      case GLFW_KEY_RIGHT:
        gAsteroidsGame->rotate(0);
        break;

        // Accelerate forward
      case GLFW_KEY_UP:
        gAsteroidsGame->engine(false);
        break;

      case GLFW_KEY_DOWN:
        break;
    }
  }
}

void frameCallback()
{  
  gDebugDraw.clearAll();
  gAsteroidsGame->updateGameStatus();

  // use built-in primitive rendering stuff
  Color colWhite(1,1,1);
  Color colGreen(0,1,0);
  //drawWorldAxis(colWhite);

  // TODO add missing rendering code.

  // draw extra data
  gAsteroidsGame->getShip()->drawEngine();
}

void initGame()
{
  // create physics scenario
  bse::phx::SceneDesc sceneDesc;
  sceneDesc.timeStep = TIME_STEP;
  sceneDesc.solverIterations = 12;
  sceneDesc.gravity = bse::Vec2(0,0);

  gScene = bse::phx::Scene::create(&sceneDesc);

  gGameSetup.ship.initialPosition = bse::Vec2(0,0);
  gGameSetup.ship.initialOrientation = 0;
  gGameSetup.ship.initialVelocity = bse::Vec2(0,0);
  gGameSetup.ship.initialAngVelocity = 0.0f; //bse_pi / 8.0f;
  gGameSetup.ship.steeringVelocity = 0.025f;
  gGameSetup.ship.enginePower = 1.0f;
  gGameSetup.ship.size = 0.3f;
  gGameSetup.ship.bulletLag = 500.0f;

  gGameSetup.ship.bullet.initialSpeed = 3.0f;
  gGameSetup.ship.bullet.size = 0.05f;

  gGameSetup.asteroids.maxEdges = 8;
  gGameSetup.asteroids.minEdges = 5;
  gGameSetup.asteroids.maxRadius = 0.25f;
  gGameSetup.asteroids.minRadius = 0.25f;
  gGameSetup.asteroids.numAsteroids = 5;
  gGameSetup.asteroids.mass = 1.0f;
  gGameSetup.spaceRadius = 2.5f;
  gGameSetup.simulationTimeStep = (1.0f / 60.0f);

  gAsteroidsGame = new AsteroidsGame(&gGameSetup);

  gAsteroidsGame->initializeLevel(gGameSetup);

  gScene->getDefaultMaterial()->setRestitution(1.0f);
  gScene->getDefaultMaterial()->setFriction(0.0f);
  gScene->setContactFeedback(gAsteroidsGame);
  gScene->setContactFilter(gAsteroidsGame);
}

void shutdownGame()
{
  delete gAsteroidsGame;
  gAsteroidsGame = 0;
  bse::phx::Scene::destroy(gScene);
  gScene = 0;
}

void initGraphics(int argc, char** argv)
{
  // create a glut window with camera management.
  RenderSceneDesc renderSceneDesc;
  renderSceneDesc.argc = argc;
  renderSceneDesc.argv = argv;
  renderSceneDesc.windowTitle = "AsteroidsBSE - SeeBeexSoft 2008-2011";
  renderSceneDesc.windowHeight = 768;
  renderSceneDesc.windowWidth = 1024;
  renderSceneDesc.windowStartX = 0;
  renderSceneDesc.windowStartY = 0;
  renderSceneDesc.updateMode = RENDER_UPDATEMODE_TIMED; // RENDER_UPDATEMODE_CONTINUOS; // RENDER_UPDATEMODE_MANUAL;
  renderSceneDesc.updateTiming = 1000 / 60; // ms
    // setup callbacks
  renderSceneDesc.keyboardFunc = keyboardCallback;
  renderSceneDesc.frameFunc = frameCallback; // paint
  renderSceneDesc.shutdownFunc = shutdownGame;

  // setup camera
  CameraDesc camera;
  camera.fixed = true;
  camera.startZoom = 2;
  camera.zoomTick = 0.5f;
  camera.zoomMax = 100.0f;
  camera.zoomMin = 0.5f;
  camera.startX = 0.0f;
  camera.startY = -1.0f;
  camera.viewTick = 0.1f;

  renderSceneDesc.camera = camera;

  initDrawUtils(renderSceneDesc);
}

int main(int argc, char** argv)
{
  initGame();

  initGraphics(argc, argv);

	return 0;
}
