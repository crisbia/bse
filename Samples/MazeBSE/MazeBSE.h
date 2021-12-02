//////////////////////////////////////////////////////////////////////////////
//
//              MazeBSE Demo - SeeBeex Soft 2008-2011
//
//  This demo shows the pathfinder capabilities to solve a maze
//
//
//
//
//
//
///////////////////////////////////////////////////////////////////////////////


// system includes
#include "SysUtils.h"

#include <vector>
#include <list>

// demo includes
#include "DrawUtils.h"

// ai includes
#include "bseGraph.h"
#include "bseAIScene.h"

// physics includes
#include "bseShape.h"
#include "bseBody.h"
#include "bseScene.h"
#include "bseTask.h"

class Maze;
class Character;
class AICharacter;
class MazeObserver;
class UpdatePathTask;

//---------------------------------------------------------------------------------------------------------------------
class MazeObserver
{
public:
  /**
   *  \brief Called when a new game starts.
  */
  virtual void onGameStarted() {}

  /**
   *  \brief Called after each game update.
   */
  virtual void onGameTicked(float deltaTime) {}

  /**
   *  \brief Called when a game finishes.
  */
  virtual void onGameFinished() {}

  /**
   *  \brief Called when player is reachd by a non-player.
  */
  virtual void onPlayerCaught(Character* prey, AICharacter* hunter) {}
};

#define MAZEOBSERVERS_CALL(observerRegistrar, observerMethod) \
  { \
    MazeObservers clients; \
    observerRegistrar->getClients(clients); \
    for (MazeObservers::iterator clientsIter = clients.begin(); clientsIter != clients.end(); ++clientsIter)  \
      (*clientsIter)->observerMethod; \
  } \

//---------------------------------------------------------------------------------------------------------------------
class MazeObserverRegistrar
{
  typedef std::list<MazeObserver*> MazeObservers;
public:
  void getClients(MazeObservers& clients);
  void registerClient(MazeObserver* observer);
  void unregisterClient(MazeObserver* observer);
private:
  MazeObservers m_observers;
};

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2* getNodePos(const bse::ai::GraphNode* node);

//---------------------------------------------------------------------------------------------------------------------
class ManhattanDistanceHeuristic : public bse::ai::GraphHeuristic
{
public:
  virtual const bse::Real estimatePathCost(const bse::ai::GraphNode* source, const bse::ai::GraphNode* dest) const
  {
    BSE_ASSERT(source && dest);

    bse::Vec2* posSource = ::getNodePos(source);
    bse::Vec2* posDest   = ::getNodePos(dest);

    bse::Vec2 diff = *posSource - *posDest;
    return 10.0f*(fabs(diff.x) + fabs(diff.y));
  }
};

//---------------------------------------------------------------------------------------------------------------------
class EulerDistanceHeuristic : public bse::ai::GraphHeuristic
{
public:
  virtual const bse::Real estimatePathCost(bse::ai::GraphNode* source, bse::ai::GraphNode* dest) const
  {
    // BSE_ASSERT(source && dest)

    bse::Vec2* posSource = ::getNodePos(source);
    bse::Vec2* posDest   = ::getNodePos(dest);

    bse::Real diff = (*posSource - *posDest).mag();
    return 10.0f*diff;
  }
};

//---------------------------------------------------------------------------------------------------------------------
class CharacterDesc
{
public:
  CharacterDesc() :
      position(0,0),
      size(0.1f)
  {
  }

  bse::Vec2 position;
  bse::Real size;
};

//---------------------------------------------------------------------------------------------------------------------
class Character
{
public:
  // debug draw flags for characters
  enum
  {
    cdb_none =            0,
    cdb_path =            1,
    cdb_closestcell =     2,
    cdb_doublecircle =    4,
    cdb_playercharacter = 8,

    cdb_all  = 0xffffffff
  };


  Character(Maze* maze, const CharacterDesc& desc);
  virtual ~Character();


  // for rendering purposes, the current path is returned
  const bse::ai::GraphPath& getPath() { return m_currentPath; }

  // Returns the current position
  bse::Vec2 getPosition() { return m_position; }

  // Forces a new position for the character
  void setPosition(const bse::Vec2& pos) { m_position = pos; }

  virtual bool update(bse::Real dt) { return false; }

  virtual void render(bse::UInt flags);

  void physicsToAI();

  const bse::Vec2& getInitialPosition() const { return m_desc.position; }

protected:
  // Stores the current path
  bse::ai::GraphPath m_currentPath;
  bse::Vec2 m_position;
  Maze* m_maze;

  // character's physics
  bse::phx::Circle* m_shape;
  bse::phx::Body*   m_body;
  const CharacterDesc m_desc;
};

//---------------------------------------------------------------------------------------------------------------------
class PlayerCharacter : public Character
{
public:
  PlayerCharacter(Maze* maze, const CharacterDesc& desc) : Character(maze, desc)
  {
    m_targetPos = desc.position;
  }

  virtual ~PlayerCharacter()
  {
  }

  void moveTargetPosition(const bse::Vec2& pos);
  void moveToTargetPosition();
  virtual bool update(bse::Real dt);
  virtual void render(bse::UInt flags);

  bse::Vec2 getTargetPosition() const { return m_targetPos; }

  void reset();
protected:
  bse::Vec2 m_targetPos;
};

//---------------------------------------------------------------------------------------------------------------------
class AICharacter : public Character
{
public:
  AICharacter(Maze* maze, const CharacterDesc& desc) :
    Character(maze, desc),
    speed(0.1f),
    m_newTargetNodeSet(false),
    m_targetNode(0)
  {
  }

  virtual ~AICharacter()
  {
  }

  void setTarget(const bse::Vec2& target);
  void setTarget(Character* target);
  void setTarget(bse::ai::GraphNode* target);

  virtual bool update(bse::Real dt);
  bool updateStep1();
  bool updateStep2(bse::Real dt);

  void initPath(bse::ai::GraphNode* initialTarget);

  void onNewTargetNodeSet(bse::ai::GraphNode* target);

  void reset();

private:
  const float speed;

  bool                m_newTargetNodeSet;
  bse::ai::GraphNode* m_targetNode;
};

//---------------------------------------------------------------------------------------------------------------------
class MazeDesc
{
public:
  MazeDesc() :
      numLogicalProcessors(1),
      numberOfRows(10),
      numberOfCols(10),
      cellSize(0.25f),
      numberOfAICharacters(1),
      aiCharactersSize(0.01f),
      playerCharacterPos(0,0),
      playerCharacterSize(0.01f)
  {

  }

  bse::UInt numLogicalProcessors;
  bse::UInt numberOfRows;
  bse::UInt numberOfCols;
  bse::Real cellSize;

  // ai characters (random position on the grid)
  bse::UInt numberOfAICharacters;
  bse::Real aiCharactersSize;     // size of the collision (and visual debug) shape

  bool createPlayerCharacter; // enable player character creation
  bse::Vec2 playerCharacterPos; // specify player character position
  bse::Real playerCharacterSize;  // size of the collision (and visual debug) shape
};

//---------------------------------------------------------------------------------------------------------------------
class Maze
{
public:
  enum
  {
    mdb_none                  = 0,
    mdb_intersections         = 1,
    mdb_connections           = 2,
    mdb_walls                 = 4,
    mdb_walls_collisionshapes = 8,
    mdb_all                   = 0xffffffff
  };

  // helper structure for prim's algorithm maze creation
  struct NodeState
  {
    enum
    {
      NODE_OUT,
      NODE_IN,
      NODE_FRONTIER,
    };

    int state;

    int row;
    int col;
    int id;

    bse::Vec2 pos;
  };



  Maze(const MazeDesc& desc, bse::TaskScheduler* taskScheduler = 0);
  ~Maze();

  // Reset the characters to their original position and the
  // restart the simulation.
  void reset(bse::TaskScheduler* taskScheduler);

  PlayerCharacter* createPlayerCharacter(const bse::Vec2& pos, bse::Real size=0.01f);
  AICharacter* createNonPlayerCharacter(const bse::Vec2& pos, bse::Real size=0.01f);

  bse::Vec2 getNodePos(bse::ai::GraphNode* node);
  void movePlayerTo(const bse::Vec2& pos);

  // Find the closest node to the character
  bse::ai::GraphNode* matchCharacterOnGraph(Character* character);
  bool computePathToTarget(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget, bool bestPathOnFail=false);
  bool computePathToTargetAsynch(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget, bool bestPathOnFail=false);
  void initPathToTarget(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget);

  void updateGame();
  void updatePhysics();

  bse::UInt getNumSearches() const { return m_numSearches; }

  PlayerCharacter* getMainPlayer() { return m_mainPlayer; }
private:
  // The pathfinder graph

  bse::ai::Graph* m_mazeGraph;
  MazeDesc  m_desc;

  bse::ai::GraphNodeData* m_nodeData;
  //bse::Vec2*          m_nodePos;
  NodeState*        m_nodeStates;

  void initPathFinderGraph(bse::UInt numRows, bse::UInt numCols, bse::Real cellSize);
  void destroyPathFinderGraph();

  void initWallsCollisionShapes();
  void initBox();

  std::vector<AICharacter*> m_npcharacters;
  std::vector<PlayerCharacter*>   m_players;
  PlayerCharacter*                m_mainPlayer;

public:
  void renderPathFinderGraph(bse::UInt debugDraw);
  void renderCharacterPaths();
  void renderCharacters();

  void clampPos(bse::Vec2& pos)
  {
    if (pos.x < 0)
      pos.x = 0;
    else if (pos.x > m_desc.cellSize * (m_desc.numberOfCols-1))
      pos.x = m_desc.cellSize * (m_desc.numberOfCols-1);

    if (pos.y < 0)
      pos.y = 0;
    else if (pos.y > m_desc.cellSize * (m_desc.numberOfRows-1))
      pos.y = m_desc.cellSize * (m_desc.numberOfRows-1);

  }

// physics
public:
  bse::phx::Scene* getPhysicsScene() const { return m_physicsScene; }
  bse::ai::AIScene* getAIScene() const { return m_aiScene; }

  bool isAsynchPathFinderEnabled() const { return m_asynchPathFinder; }
  void setAsynchPathFinder(bool enable = true);

private:
  bool m_asynchPathFinder;
private:
  bse::phx::Scene     *m_physicsScene;
  bse::ai::AIScene    *m_aiScene;
  bse::TaskScheduler  *m_taskScheduler;

  bse::FrameObjectsPool<UpdatePathTask, 16> m_updatePathTasks;
private:
  int getNodeState(bse::ai::GraphNode* node);
  void setNodeState(bse::ai::GraphNode* node, int state);
  void getNodeNeighbours(bse::ai::GraphNode* node, int* nbs);
  void setNodeNeighboursToFrontier(bse::ai::GraphNode* node, std::vector<bse::ai::GraphNode*>& frontier);

  bse::UInt m_numSearches;
};

//---------------------------------------------------------------------------------------------------------------------
class UpdatePathTask : public bse::Task
{
public:
  UpdatePathTask() {}
  void setup(
    Maze *maze,
    bse::ai::GraphNode* source,
    bse::ai::GraphNode* target,
    bse::ai::GraphPath* pathToTarget, bool bestPathOnFail=false);

  virtual void compute();

  bool getSearchResult() const { return m_lastSearchResult; }

private:
  Maze *m_maze;
  bse::ai::GraphNode* m_source;
  bse::ai::GraphNode* m_target;
  bse::ai::GraphPath* m_pathToTarget;
  bool m_bestPathOnFail;

  bool m_lastSearchResult;
};
