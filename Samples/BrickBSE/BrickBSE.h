// system includes
#include <vector>

// demo utils
#include "DrawUtils.h"

// bse includes
#include "bsePhysics.h"

namespace bse
{
namespace ai
{
  class KinematicAgent;
}
}

class BricksGame; // FW decl


typedef enum {
  INVALID_BRICKSOBJ = -1,
  BRICKSOBJ_WALL,
  BRICKSOBJ_BALL,
  BRICKSOBJ_BRICK,
  BRICKSOBJ_PADDLE,
  NUM_BRICKSOBJ_TYPES
} BricksObjType;


class BricksObj
{
public:
  BricksObjType type;
  bse::phx::Body* body;

protected:
  BricksObj(BricksGame* game, BricksObjType t) : type(t)
  {
    this->game = game;
  }
  virtual ~BricksObj();
  BricksGame* game;
};


class BrickDesc
{
public:
  bse::UInt initialHitsToDestroy; // set UNLIMITED_HITS for indestructible bricks

  // during description, we want to know which brick is this.
  bse::UInt column;
  bse::UInt row;
  bse::Vec2 dims;
  bse::Vec2 pos;
  Color   color;
};

class Brick : public BricksObj
{
public:
  Brick(BricksGame* game, const BrickDesc& d);
  void render();

  BrickDesc desc;
  int       hitsToDestroy;
  bse::UInt   brickIndex; // useful index in the vector of bricks
};


class BallDesc
{
public:
  bse::Real size;
  bse::Vec2 pos;
  bse::Real initialVel;
};

class Ball : public BricksObj
{
public:
  Ball(BricksGame* game, const BallDesc& d);
  void render();

  BallDesc desc;
  bool onPaddle;
};


class PaddleDesc
{
public:
  bse::Vec2 dims;
  bse::Vec2 pos;
  bse::Real speed;
  bse::Real normalDeviationFactor;
};

class Paddle : public BricksObj
{
public:
  Paddle(BricksGame* game, const PaddleDesc& d);
  void render();

  PaddleDesc desc;
  bool hasBall;
};

typedef enum {
  INVALID_WALL = -1,
  WALL_DOWN,
  WALL_UP,
  WALL_LEFT,
  WALL_RIGHT,
  NUM_WALLS
} WallType;

class WallDesc
{
public:
  bse::Vec2   dims;
  bse::Vec2   pos;
  WallType  id;
};

class Wall : public BricksObj
{
public:
  Wall(BricksGame* game, const WallDesc& d);
  WallDesc desc;
  void render();
};

class BricksGameDesc
{
public:
  bse::UInt                 difficulty;
  bse::UInt                 numColumns;
  bse::UInt                 numRows;
  bse::Real                 levelWidth;
  bse::Real                 levelHeight;
  bse::Real                 startX;
  bse::Real                 startY;

  bse::UInt                 initialPlayerLives; // set UNLIMITED_LIVES for infinite lives

  std::vector<BrickDesc>  bricksDescs;
  PaddleDesc              paddleDesc;
  BallDesc                ballDesc;
};


class BricksGame : 
    public bse::phx::ContactFilter, 
    public bse::phx::ContactFeedback
{
public:
  BricksGame(const BricksGameDesc& d);
  ~BricksGame();

  void throwBall();
  void removeBricks();
  void updateGameStatus();
  void resetPaddle();
  void movePaddle(bse::Real move);
  bse::UInt getNumberOfLives() const { return playerLives; }

  virtual bool applyFilterOnContact(bse::phx::Contact* contact);
  virtual void onContactReport(const bse::phx::Contact* contact);

  void renderGame();

protected:
  void createLevelRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY);
  void initGame();

  void updatePaddleAgent(const bse::Vec2& targetPos, const bse::Vec2& targetVel);

protected:

  BricksGameDesc desc;
  bse::phx::Scene* physicsScene;

  bse::ai::KinematicAgent* paddleAgent;
  bse::ai::KinematicAgent* ballAgent;

  std::vector<Brick*> bricks;
  bse::UInt numberOfBricks;


public:
  Ball* ball;
  Wall* walls[4];
  Paddle* paddle;

  // game status
  bse::UInt playerLives;
  bool gameOver;
  bool lifeLost;
  bool aiPaddle; // ai demo. if enabled the ai tries to follow the ball

  std::vector<Brick*> bricksToRemove; // filled by the contact reporting system

public:
  bse::phx::Scene* getPhysicsScene() const { return physicsScene; }
private:
  void createRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY);

  bse::GenericTimer ballFlightTimer;
};