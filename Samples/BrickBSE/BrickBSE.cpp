//////////////////////////////////////////////////////////////////////////////
//
//              BrickBSE Demo - SeeBeex Soft 2008-2011
//
//  This very simple demo is a mini game, a clone of the classic Bricks/Arkanoid
//  It's not a complete game. The purpose is just to show some of the physics
//  capability of the engine. Also, a simple AI steering behaviour is used
//  to automatically move the palette toward the ball.
//
///////////////////////////////////////////////////////////////////////////////

#include <vector>

// bse includes
#include "bsePhysics.h"
#include "bseKinematicAgent.h"

#include "BrickBSE.h"

// settings
const bse::Real TIME_STEP = 1.0f / 60.0f;
// fill the level with bricks
const bse::UInt NUM_ROWS = 5;
const bse::UInt NUM_COLUMNS = 5;
const bse::UInt HITS_TO_DESTROY = 2;
const bse::Real ROOM_WIDTH = 3;
const bse::Real ROOM_HEIGHT = 3;
const bse::Real ROOM_START_X = -ROOM_WIDTH / 2.0f;
const bse::Real ROOM_START_Y = -ROOM_HEIGHT / 2.0f;
const bse::Real PERC_HEIGHT = 0.25f; // percentage of the window height occupied by bricks
const bse::Real BRICK_WIDTH   = ROOM_WIDTH / NUM_COLUMNS;
const bse::Real BRICK_HEIGHT  = ROOM_HEIGHT*PERC_HEIGHT / NUM_ROWS;

const bse::Real PADDLE_WIDTH = ROOM_WIDTH*0.2f;
const bse::Real PADDLE_HEIGHT = PADDLE_WIDTH*0.25f;
const bse::Real PADDLE_SPEED = 0.05f;

const bse::Real BALL_SIZE = 0.1f;
const bse::Real INITIAL_BALL_VEL = 2.0f;

const int INITIAL_PLAYER_LIVES = 3;

const bse::UInt UNLIMITED_LIVES = (bse::UInt)-1;
const bse::UInt UNLIMITED_HITS = (bse::UInt)-1;

// this constant defines how much the contact normal is deviated from the vertical
// to create a controllable non realistic effect in the bounce.
const bse::Real NORMAL_DEVIATION_FACTOR = 0.5f;

// simple graphics container
RenderScene* gRenderScene = 0;

PhysicsDebugDraw gDebugDraw;

BricksObj::~BricksObj()
{
  game->getPhysicsScene()->releaseBody(body);
}


Brick::Brick(BricksGame* game, const BrickDesc& d) : BricksObj(game, BRICKSOBJ_BRICK), desc(d)
{
  bse::phx::BoxDesc boxDesc;
  boxDesc.dims = desc.dims;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_STATIC;
  bodyDesc.shapesDescs.push_back(&boxDesc);

  bodyDesc.position = desc.pos;

  body = game->getPhysicsScene()->createBody(bodyDesc);
  body->setUserData(this);

  hitsToDestroy = desc.initialHitsToDestroy;
}

void Brick::render()
{
  bse::phx::ShapesList &shapes = body->getShapes();
  bse::phx::Box* box = (bse::phx::Box*)shapes[0];
  bse::Vec2 halfDims = box->getHalfDims();
  halfDims.x *= 0.95f;
  halfDims.y *= 0.9f;
  gDebugDraw.drawBox(body->getPosition(), halfDims, desc.color, true);
  Color border = desc.color.lighten(5);
  gDebugDraw.drawBox(body->getPosition(), halfDims, border, false);
}


Paddle::Paddle(BricksGame* game, const PaddleDesc& d) : BricksObj(game, BRICKSOBJ_PADDLE), desc(d)
{
  // create movable paddle
  // e.g. kinematic body,
  // 1. it's moved using moveTo, that preserves the velocity, or using setVelocity
  // 2. it can trepass statics, but it doesn't react to collision with them
  // 3. it's not affected by collision, but it generates impulses on dynamic bodies

  bse::phx::BoxDesc shapeDesc;
  shapeDesc.dims = desc.dims;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_STATIC;
  // store the pointer to the shape descriptor. the shape is automatically created internally.
  bodyDesc.shapesDescs.push_back(&shapeDesc);
  body = game->getPhysicsScene()->createBody(bodyDesc);

  // center the paddle
  body->setPosition(desc.pos);

  // store a useful pointer in the game stuff
  body->setUserData(this);
}

void Paddle::render()
{
  Color colPalette(0.5f, 0.5f, 0.5f);
  gDebugDraw.drawBox(body->getPosition(), desc.dims*0.5f, colPalette, true);
}

Ball::Ball(BricksGame* game, const BallDesc& d) : BricksObj(game, BRICKSOBJ_BALL), desc(d)
{
  // create the ball
  bse::phx::BodyDesc bodyDesc;
  bodyDesc.position = desc.pos;
  bodyDesc.mass = 1;
  bodyDesc.inertia = 0.1f;

  bse::phx::CircleDesc shapeDesc;
  shapeDesc.radius = desc.size / 2.0f;

  bodyDesc.shapesDescs.push_back(&shapeDesc);
  body = game->getPhysicsScene()->createBody(bodyDesc);
  body->setUserData(this);
}

void Ball::render()
{
  Color colBall(0.8f,0.8f,0.8f);
  gDebugDraw.drawCircle(body->getPosition(), desc.size/2.0f, 12, colBall, true);
}

Wall::Wall(BricksGame* game, const WallDesc& d) : BricksObj(game, BRICKSOBJ_WALL), desc(d)
{
  bse::phx::BoxDesc boxDesc;
  boxDesc.dims = desc.dims;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_STATIC;
  bodyDesc.shapesDescs.push_back(&boxDesc);
  body = game->getPhysicsScene()->createBody(bodyDesc);
  body->setPosition(desc.pos);
  body->setUserData(this);
}

void Wall::render()
{
  bse::Vec2 halfDims = desc.dims*0.5f;
  if (desc.id == WALL_DOWN || desc.id == WALL_UP)
  {
    // cover the corners
    halfDims.x += 2*halfDims.y;
  }

  Color colWalls(1,1,1);
  gDebugDraw.drawBox(desc.pos, halfDims, colWalls, true);
}

BricksGame::BricksGame(const BricksGameDesc& desc)
{
  this->desc = desc;

  // create physics scenario
  bse::phx::SceneDesc sceneDesc;
  sceneDesc.timeStep = TIME_STEP;
  sceneDesc.solverIterations = 12;
  sceneDesc.gravity = bse::Vec2(0,0);

  physicsScene = bse::phx::Scene::create(&sceneDesc);
  physicsScene->getDefaultMaterial()->setRestitution(1.0f);
  physicsScene->getDefaultMaterial()->setFriction(0.0f);
  physicsScene->setContactFeedback(this);
  physicsScene->setContactFilter(this);

  // build the operation space
  createRoom(physicsScene, desc.startX, desc.startY, desc.levelWidth, desc.levelHeight);


  // create a bunch of bricks, all of them need 2 hits to be destroyed
  numberOfBricks = (bse::UInt)desc.bricksDescs.size();
  for (bse::UInt iBrick = 0; iBrick < numberOfBricks; ++iBrick)
  {
    Brick* brick = new Brick(this, desc.bricksDescs[iBrick]);
    brick->brickIndex = iBrick;
    bricks.push_back(brick);
  }

  // create the ball
  ball = new Ball(this, desc.ballDesc);

  // create the paddle
  paddle = new Paddle(this, desc.paddleDesc);

  ball->onPaddle = true;
  paddle->hasBall = true;

  playerLives = desc.initialPlayerLives;
  gameOver = (playerLives==0);

  lifeLost = false;
  aiPaddle = false;

  paddleAgent = new bse::ai::KinematicAgent();
  paddleAgent->lockY(); // can't move on Y

  ballAgent = new bse::ai::KinematicAgent();
}

BricksGame::~BricksGame()
{
  delete ball;
  delete paddle;
  for (int i=0; i<4; ++i)
    delete walls[i];

  for (size_t iBrick=0; iBrick<bricks.size(); ++iBrick)
  {
    delete bricks[iBrick]; // safe, when I remove a brick, i set the pointer to 0
  }

  delete paddleAgent;
  delete ballAgent;

  bse::phx::Scene::destroy(physicsScene);
}

void BricksGame::throwBall()
{
  // if the palette has the ball, then release it with a upward velocity
  if (paddle->hasBall)
  {
    bse::Vec2 vel(0.5f, 1.0f);
    vel.normalize();
    vel *= ball->desc.initialVel;

    ball->body->setLinearVelocity(vel);

    // detach the ball
    paddle->hasBall = false;
    ball->onPaddle = false;

    ballFlightTimer.startTimer();
  }
}

void BricksGame::removeBricks()
{
  // remove bodies for bricks that have been hit.
  for (size_t i=0; i<bricksToRemove.size(); ++i)
  {
    --numberOfBricks;

    bse::UInt index = bricksToRemove[i]->brickIndex;
    delete bricks[index];
    bricks[index] = 0;
  }

  bricksToRemove.clear();

  if (numberOfBricks == 0)
  {
    resetPaddle();
    gameOver = true;
  }
}

void BricksGame::updateGameStatus()
{
  // set the target of the paddle agent
  updatePaddleAgent(ball->body->getPosition(), ball->body->getLinearVelocity());

    // if the ball is still on the palette, update it's position
  if (ball->onPaddle)
  {
    bse::Vec2 palettePos = paddle->body->getPosition();
    ball->body->setPosition(palettePos.x, ball->desc.pos.y);
  }
  else
  {
    // increase the speed of the ball
    if (ballFlightTimer.getElapsedTime() > 5000.0f/(1+0.1f*desc.difficulty))
    {
      ballFlightTimer.stopTimer();
      bse::Vec2 vel = ball->body->getLinearVelocity();
      bse::Vec2 newVel = vel*1.25f;
      if (newVel.mag() < 5.0f)
        ball->body->setLinearVelocity(vel);
      ballFlightTimer.startTimer();
    }
  }

  // update status to check if the game is over
  if (!gameOver)
  {
    // check if contact between the ball and the lower wall happened.
    if (lifeLost)
    {
      lifeLost = false;
      if (desc.initialPlayerLives != UNLIMITED_LIVES)
        playerLives--;

      // if the number of lives is 0, stop the game
      gameOver = (playerLives == 0);

      // if the game is not over, reset palette position and put the ball on hit. it will be following it next frame
      resetPaddle();
      if (!gameOver)
      {
        if (aiPaddle)
          throwBall();
      }
    }
  }

  if (!gameOver)
  {
    getPhysicsScene()->simulate(TIME_STEP);
    removeBricks();
  }
}

void BricksGame::resetPaddle()
{
  ball->onPaddle = true;
  paddle->hasBall = true;
  bse::Vec2 paddlePos = paddle->desc.pos;
  paddle->body->setPosition(paddlePos);
  ball->body->setPosition(ball->desc.pos);
  ballFlightTimer.stopTimer();
}

void BricksGame::movePaddle(bse::Real move)
{
  bse::Vec2 pos = paddle->body->getPosition();

  pos.x += move;

  // avoid the palette moving through the walls
  const bse::Real lowerLimit = desc.startX + paddle->desc.dims.x/2.0f;
  const bse::Real upperLimit = desc.startX + desc.levelWidth - paddle->desc.dims.x / 2.0f;

  if (pos.x < lowerLimit)
    pos.x = lowerLimit;
  if (pos.x > upperLimit)
    pos.x = upperLimit;

  paddle->body->setPosition(pos);
}

// contact filter
// this advanced filter is here used to modify the contact normal
// when the ball hits the palette, to create an "unphysical" effect
// of bounce
bool BricksGame::applyFilterOnContact(bse::phx::Contact* contact)
{
  // check the contact, if its a contact ball/palette and change the normal accordingly
  // to a deviation factor

  bse::UserData uData1 = contact->body1->getUserData();
  bse::UserData uData2 = contact->body2->getUserData();

  if (uData1==0 || uData2==0)
    return false;

  BricksObj* aObj1 = static_cast<BricksObj*>(uData1);
  BricksObj* aObj2 = static_cast<BricksObj*>(uData2);

  if ((aObj1->type == BRICKSOBJ_BALL && aObj2->type == BRICKSOBJ_PADDLE) ||
      (aObj2->type == BRICKSOBJ_BALL && aObj1->type == BRICKSOBJ_PADDLE) )
  {
    // change the normal (now it's vertical)
    bse::Vec2 palettePos = paddle->body->getPosition();

    // determine if the contact point is on the palette surface (the upper edge of the box)
    if (contact->contactPoint.y > palettePos.y + paddle->desc.dims.y/2.0f - 0.0001f)
    {
      bse::Real xN = (contact->contactPoint.x-palettePos.x) / (paddle->desc.dims.x/2.0f) * paddle->desc.normalDeviationFactor;
      bse::Vec2 deviatedNormal = bse::Vec2(xN, contact->contactNormal.y);
      if (deviatedNormal.y<0)
        deviatedNormal.x = -deviatedNormal.x;

      deviatedNormal.normalize();
      contact->contactNormal = deviatedNormal;

      return false;
    }
  }

  return false;
}

void BricksGame::onContactReport(const bse::phx::Contact* contact)
{
  // check the contact, if its a contact ball/brick, remove the brick from the scene
  // ATTENTION: the brick can't be removed NOW. It must be queued and removed later
  // remember that here we are still INSIDE the simulation step.

  bse::UserData uData1 = contact->body1->getUserData();
  bse::UserData uData2 = contact->body2->getUserData();

  BricksObj* aObj1 = 0;
  BricksObj* aObj2 = 0;

  if (uData1)
    aObj1 = static_cast<BricksObj*>(uData1);
  if (uData2)
    aObj2 = static_cast<BricksObj*>(uData2);

  if (aObj1 && aObj1->type == BRICKSOBJ_BRICK)
  {
    Brick* brick = static_cast<Brick*>(aObj1);
    brick->hitsToDestroy--;
    if (brick->hitsToDestroy == 0)
      bricksToRemove.push_back(brick);
  }
  if (aObj2 && aObj2->type == BRICKSOBJ_BRICK)
  {
    Brick* brick = static_cast<Brick*>(aObj2);
    if (brick->desc.initialHitsToDestroy != UNLIMITED_HITS)
    {
      brick->hitsToDestroy--;
    }

    if (brick->hitsToDestroy == 0)
      bricksToRemove.push_back(brick);
  }

  if (aObj1 &&
      aObj2 &&
      (
          (aObj1->type == BRICKSOBJ_BALL && aObj2->type == BRICKSOBJ_WALL) ||
          (aObj2->type == BRICKSOBJ_BALL && aObj1->type == BRICKSOBJ_WALL)
      ))
  {
      Wall* wall;
      if (aObj1->type == BRICKSOBJ_WALL)
      {
        wall = static_cast<Wall*>(aObj1);
      }
      else
      {
        wall = static_cast<Wall*>(aObj2);
      }

      // the ball hit the bottom of the world
      // a life is consumed
      if (wall->desc.id == WALL_DOWN)
      {
        lifeLost = true;
      }
  }
}

// instead of using the auto rendering for debug bodies-shapes, render them
// more nicely
void BricksGame::renderGame()
{
  const Color cols[2] =
  {
    Color(0,1,0),
    Color(1,0,0)
  };

  for (unsigned int row = 0; row < NUM_ROWS; ++row)
  {
    unsigned int offset = row % 2;
    for (unsigned int col = 0; col < NUM_COLUMNS; ++col)
    {
      unsigned int index = row*NUM_COLUMNS+col;
      if (!bricks[index])
        continue; // destroyed brick

      Color color = cols[(offset + col)%2];
      bricks[index]->render();
    }
  }

  ball->render();

  paddle->render();

  for (int i=0; i<4; ++i)
  {
    walls[i]->render();
  }
}

//////////////////////////////////
// AI experiments
// A kinematic agent moves synchronously with the ball, it doesn't have a real behaviour
// because it's position is updated with the position of the rigid body of the ball
// Another kinematic agent is attached to the palette. This one, when the AI is enables,
// force the palette to move toward the ball.

//Try to move toward the target
void BricksGame::updatePaddleAgent(const bse::Vec2& targetPos, const bse::Vec2& targetVel)
{
  if (paddle->hasBall)
    return;
  if (!aiPaddle)
    return;

  ballAgent->updatePosition(targetPos);
  ballAgent->updateVelocity(targetVel);

  bse::Vec2 palettePos = paddle->body->getPosition();

  if (fabs(palettePos.x-targetPos.x)<0.1f)
    return;

  paddleAgent->updatePosition(palettePos);
  paddleAgent->updateVelocity(bse::Vec2(0,0)); // the palette is considered with 0 velocity

  paddleAgent->pursue(ballAgent, TIME_STEP, 0.2f);
  bse::Vec2 agentPos = paddleAgent->getPosition();

  bse::Real displX = agentPos.x - palettePos.x;

  // move only if the ball is not too close
  //if (fabs(palettePos.y - targetPos.y)>0.1f)
  {
    if (displX > paddle->desc.speed)
      displX = paddle->desc.speed;
    else
      if (displX < -paddle->desc.speed)
        displX = -paddle->desc.speed;

    movePaddle(displX);
  }
}

// x, y is the lower left corner
void BricksGame::createRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY)
{
  WallDesc x_wDesc;
  x_wDesc.dims.x = sizeX;
  x_wDesc.dims.y = 0.1f; // just a bit of thickness

  WallDesc y_wDesc;
  y_wDesc.dims.x = 0.1f;
  y_wDesc.dims.y = sizeY;

  WallDesc wDesc[4];

  // lower bound
  {
    wDesc[WALL_DOWN] = x_wDesc;
    wDesc[WALL_DOWN].id = WALL_DOWN;
    wDesc[WALL_DOWN].pos = bse::Vec2(x + sizeX / 2.0f, y-0.05f);
  }

  // higher bound
  {
    wDesc[WALL_UP] = x_wDesc;
    wDesc[WALL_UP].id = WALL_UP;
    wDesc[WALL_UP].pos = bse::Vec2(x + sizeX / 2.0f, y+0.05f + sizeY);
  }

  // left bound
  {
    wDesc[WALL_LEFT] = y_wDesc;
    wDesc[WALL_LEFT].id = WALL_LEFT;
    wDesc[WALL_LEFT].pos = bse::Vec2(x-0.05f, y + sizeY/2.0f);
  }

  // right bound
  {
    wDesc[WALL_RIGHT] = y_wDesc;
    wDesc[WALL_RIGHT].id = WALL_RIGHT;
    wDesc[WALL_RIGHT].pos = bse::Vec2(x+0.05f + sizeX, y + sizeY/2.0f);
  }

  for (int i=0; i<4; ++i)
  {
    walls[i] = new Wall(this, wDesc[i]);
  }
}




// global game object
BricksGame* gBricksGame = 0;
void initGame();
void shutDownGame();

void keyboardCallback(int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
  {
    if (!gBricksGame->aiPaddle && !gBricksGame->gameOver)
    {
      switch (key)
      {
        case GLFW_KEY_LEFT:
          gBricksGame->movePaddle(-PADDLE_SPEED);
        break;
        case GLFW_KEY_RIGHT:
          gBricksGame->movePaddle(PADDLE_SPEED);
          break;
      }
    }

    switch (key)
    {
    case GLFW_KEY_SPACE:
      {
        if (!gBricksGame->gameOver)
        {
          gBricksGame->throwBall();
        }
        else
        {
          // restart
          shutDownGame();
          initGame();
        }
      }
      break;
    case GLFW_KEY_Q:
      gBricksGame->aiPaddle = !gBricksGame->aiPaddle;
      break;
    }
  }
}

TextManager gTextManager;

void frameCallback()
{
  gDebugDraw.clearAll();

  gBricksGame->updateGameStatus();

  // use built in primitive rendering stuff
  gTextManager.startDrawing(5,20, 20); // startX, starY, offset

  const Color lightRed(0.9f,0.6f,0.5f);
  const Color lightBlue(0.3f,0.9f,0.3f);

  if (gBricksGame->aiPaddle)
  {
    gTextManager.drawText(lightRed, "q : disable AI paddle control");
  }
  else
  {
    gTextManager.drawText(lightRed, "q : enable AI paddle control");
  }

  gTextManager.drawText(lightRed, "SPACE : throw the ball");
  gTextManager.drawText(lightRed, "LEFT/RIGHT : control the paddle");

  if (!gBricksGame->gameOver)
  {
    if (gBricksGame->getNumberOfLives() == UNLIMITED_LIVES)
      gTextManager.drawText(lightBlue, "unlimited lives");
    else
      gTextManager.drawText(lightBlue, "lives: %d", gBricksGame->getNumberOfLives());
  }
  else
  {
    gTextManager.drawText(lightBlue, "Game Over");
  }


  if (gBricksGame)
    gBricksGame->renderGame();

  gDebugDraw.renderAll();
}

void initGame()
{
  BricksGameDesc gameDesc;

  gameDesc.initialPlayerLives = 3; //UNLIMITED_LIVES;
  gameDesc.difficulty = 0; // the higher, the more often ball speed increases

  PaddleDesc paddleDesc;
  paddleDesc.dims.x = PADDLE_WIDTH;
  paddleDesc.dims.y = PADDLE_HEIGHT;
  paddleDesc.pos.x = ROOM_START_X + ROOM_WIDTH/2.0f;
  paddleDesc.pos.y = ROOM_START_Y + PADDLE_HEIGHT/2.0f + 0.1f;
  paddleDesc.speed = PADDLE_SPEED;
  paddleDesc.normalDeviationFactor = NORMAL_DEVIATION_FACTOR;

  gameDesc.paddleDesc = paddleDesc;

  BallDesc ballDesc;
  ballDesc.pos = paddleDesc.pos;
  ballDesc.pos.y += PADDLE_HEIGHT/2.0f + BALL_SIZE/2.0f;
  ballDesc.size = BALL_SIZE;
  ballDesc.initialVel = INITIAL_BALL_VEL;

  gameDesc.ballDesc = ballDesc;



  const Color colors[2] =
  {
    Color(0,1,0),
    Color(1,0,0)
  };

  BrickDesc brickDesc;
  brickDesc.initialHitsToDestroy = 2; // UNLIMITED_HITS;
  brickDesc.dims.x = BRICK_WIDTH;
  brickDesc.dims.y = BRICK_HEIGHT;

  bse::Real brickX = ROOM_START_X + BRICK_WIDTH/2.0f;
  bse::UInt brickIndex = 0;
  for (bse::UInt column = 0; column < NUM_COLUMNS; ++column)
  {
    int offset = column % 2;
    bse::Real brickY = ROOM_START_Y + ROOM_HEIGHT - BRICK_HEIGHT / 2.0f;
    for (bse::UInt row = 0; row < NUM_ROWS; ++row)
    {
      Color color = colors[(offset + row)%2];
      BrickDesc desc = brickDesc;
      desc.pos.x = brickX;
      desc.pos.y = brickY;
      desc.column = column;
      desc.row = row;
      desc.color = color;

      gameDesc.bricksDescs.push_back(desc);

      brickY -= BRICK_HEIGHT;
      brickIndex++;
    }
    brickX += BRICK_WIDTH;
  }

  gameDesc.levelHeight = ROOM_HEIGHT;
  gameDesc.levelWidth = ROOM_WIDTH;
  gameDesc.startX = ROOM_START_X;
  gameDesc.startY = ROOM_START_Y;

  gBricksGame = new BricksGame(gameDesc);
}

void shutDownGame()
{
  delete gBricksGame;
  gBricksGame = 0;
}

void initGraphics(int argc, char** argv)
{
  // create a glut window with camera management.
  RenderSceneDesc renderSceneDesc;
  renderSceneDesc.argc = argc;
  renderSceneDesc.argv = argv;
  renderSceneDesc.windowTitle = "BrickBSE - SeeBeexSoft 2009";
  renderSceneDesc.windowHeight = 480;
  renderSceneDesc.windowWidth = 1024;
  renderSceneDesc.windowStartX = 0;
  renderSceneDesc.windowStartY = 0;
  renderSceneDesc.updateMode = RENDER_UPDATEMODE_TIMED; // RENDER_UPDATEMODE_CONTINUOS; // RENDER_UPDATEMODE_MANUAL;
  renderSceneDesc.updateTiming = 1000 / 60; // ms
  renderSceneDesc.camera.fixed = true;
    // setup callbacks
  renderSceneDesc.keyboardFunc = keyboardCallback;
  renderSceneDesc.frameFunc = frameCallback;
  renderSceneDesc.shutdownFunc = shutDownGame;

  initDrawUtils(renderSceneDesc);
}

#include <string>
#include <stdio.h>

int main(int argc, char** argv)
{
#if 0
// Sound effects simple experiments.
  FILE* soundFile = 0;
  errno_t err = fopen_s(&soundFile, "D:\\Projects\\GamesProg\\bseDevelopmentOSVN\\bin\\assets\\pongblip1.wav", "r+");
  fseek(soundFile, 0, SEEK_END);
  unsigned long inSize = ftell(soundFile);
  char* data = new char[inSize];
  fseek(soundFile, 0, SEEK_SET);
  fread(data, inSize, 1, soundFile);

  fclose(soundFile);
  HMODULE hMod = ::GetModuleHandle(NULL);
//  BOOL res = PlaySound(L"assets\\pongblip1.wav", hMod, SND_FILENAME);
  BOOL res = PlaySound((wchar_t*)data, hMod, SND_MEMORY);
#endif

  initGame();

  initGraphics(argc, argv);

  shutDownGame();

	return 0;
}
