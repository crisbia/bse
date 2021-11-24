// system includes
#include <vector>

// bse includes
#include "bsePhysics.h"

class Bullet;
class Ship;
class Asteroid;

typedef enum {
  INVALID_GAMEOBJ = -1,
  GAMEOBJ_SHIP,
  GAMEOBJ_BULLET,
  GAMEOBJ_ASTEROID,
  NUM_GAMEOBJ_TYPES
} GameObjType;


class GameObj
{
public:
  GameObj(GameObjType t);
  virtual ~GameObj();
  bse::phx::AABB getBounds() const;
  void setPose(const bse::Vec2& position, const bse::Real orientation);
  void getPose(bse::Vec2& position, bse::Real& orientation) const;
  void setVelocities(const bse::Vec2& velocity, const bse::Real rotation);
  void getVelocities(bse::Vec2& velocity, bse::Real rotation) const;
  void registerUserData();

public:
  GameObjType type;
protected:
  bse::phx::Body* m_body;
  bse::phx::Shape* m_shape;

public:
  const bse::phx::Shape* getShape() const { return m_shape; }

public:
  void disableCollisionDetection(const GameObj* obj) const;
};


class BulletSetup
{
public:
  float initialSpeed;
  float size;
  // fields filled by the ship when it shoots
  bse::Vec2 direction;
  bse::Vec2 position;
};

class ShipSetup
{
public:
  bse::Real size;
  bse::Vec2 initialPosition;
  bse::Real initialOrientation;
  bse::Vec2 initialVelocity;
  bse::Real initialAngVelocity;
  bse::Real bulletLag;          // time that must pass between two bullets
  bse::Real enginePower;
  bse::Real steeringVelocity;
  int     numberOfLives;
  BulletSetup bullet;
};

class AsteroidsSetup
{
public:
  int numAsteroids;
  float minRadius;
  float maxRadius;
  int   minEdges;
  int   maxEdges;
  float mass;
};

class AsteroidDesc
{
public:
  int numberOfEdges;
  float externalRadius;
  bse::Vec2 position;
  bse::Real orientation;
  bse::Vec2 velocity;
  bse::Real rotation;

  // this param decides the number of pieces in which the asteroid will break into
  // 0: must disappear
  // 1: undestructable
  // >1: breakable in some pieces
  int numberOfPieces;
  float mass;
};


// Ship, physically controlled
class Ship : public GameObj
{
public:
  Ship(ShipSetup* setup);
  virtual ~Ship();

  void drawEngine() const;
  void applyEnginePower(const bool powerOn);
  void applyRotation(const int rotate);
  Bullet* shootBullet();

  bse::GenericTimer m_shootTimer;

protected:
  ShipSetup m_setup;
  bool m_powerOn;
};

class Bullet : public GameObj
{
public:
  Bullet(BulletSetup* setup);
  virtual ~Bullet();

public:
  int indexInList; // to speed up removal
  BulletSetup m_setup;
};

class Asteroid : public GameObj
{
public:
  Asteroid(AsteroidDesc* desc);
  virtual ~Asteroid();

  void saveStatusToDesc(AsteroidDesc& desc);
  
public:
  int indexInList; // to speed up removal
protected:
  AsteroidDesc m_desc;
};

template <class T>
class List : public std::vector<T>
{
public:
  bool removeFromList(T& obj)
  {
    // obj must have a member indexInList to speed up the access
    int numObjs = static_cast<int>(this->size());
    if (obj->indexInList < numObjs-1)
    {
      int tempIndex = obj->indexInList;
      T temp = this->at(numObjs-1);
      this->at(numObjs-1) = obj;
      this->at(tempIndex) = temp;
      this->at(tempIndex)->indexInList = tempIndex;
    }
    this->pop_back();
    return true;
  }
  void insertInList(T& obj)
  {
    this->push_back(obj);
    obj->indexInList = static_cast<int>(this->size()-1);
  }
};

typedef List<Bullet*> BulletsList;
typedef List<Bullet*>::iterator BulletsListIter;
typedef List<Asteroid*> AsteroidsList;
typedef List<Asteroid*>::iterator AsteroidsListIter;

class AsteroidsGameSetup
{
public:
  ShipSetup ship;
  AsteroidsSetup asteroids;
  float spaceRadius;
  float simulationTimeStep;
};

class CollisionPair
{
public:
  // it just register the collision between two game objects
  GameObj* obj1;
  GameObj* obj2;
};

class AsteroidsGame : 
  public bse::phx::ContactFilter, 
  public bse::phx::ContactFeedback
{
public:
  AsteroidsGame(AsteroidsGameSetup* gameSetup);
  virtual ~AsteroidsGame();
  void initializeLevel(AsteroidsGameSetup& gameSetup);
  bool collisionBulletAsteroid(GameObj* obj1, GameObj* obj2, Bullet* &bullet, Asteroid* &asteroid);
  bool collisionShipAsteroid(GameObj* obj1, GameObj* obj2, Ship* &ship, Asteroid* &asteroid);
  void updateGameStatus();
  const Ship* getShip() const { return m_ship; }
// Level management  
protected:
  AsteroidsGameSetup m_gameSetup;
  bool m_levelReady;

// Ship
public:
  void engine(const bool on);
  void rotate(const int rot);
  void shoot(const bool sh);
  
protected:
  void resetShip();
  Ship* m_ship;
  bool m_engineOn;
  int m_rotate;
  bool m_shoot;

// Bullets
protected:
  void emptyBulletsList();
  bool removeBullet(Bullet* bullet);
  void insertBullet(Bullet* bullet);
  BulletsList    m_bullets;

// Asteroids
protected:
  void emptyAsteroidsList();
  bool removeAsteroid(Asteroid* asteroid);
  void insertAsteroid(Asteroid* asteroid);  
  AsteroidsList m_asteroids;

  bool applyFilterOnContact(bse::phx::Contact* contact);
  void onContactReport(const bse::phx::Contact* contact);
protected:
  std::list<CollisionPair> m_collisionEvents;

};

