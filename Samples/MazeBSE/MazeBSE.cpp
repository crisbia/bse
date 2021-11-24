//////////////////////////////////////////////////////////////////////////////
//
//              MazeBSE Demo - SeeBeex Soft 2008-2011
//
//  This demo shows the pathfinder capabilities to solve a maze
//  A simple perfect maze creation algorithm is used to build a
//  path finding graph, then a bunch of hunters is set randomly in the maze.
//  They all perform a path finding with goal=player.
//  The player can move through the maze, but it can't violate the walls,
//  because the collision is enabled and the player movement is dynamically
//  based, not kinematic.
///////////////////////////////////////////////////////////////////////////////


// system includes
#include "SysUtils.h"

#include <vector>

// glut includes
#include <glut.h>

// demo includes
#include "DrawUtils.h"

#include "bseGraph.h"
#include "bseAIScene.h"
#include "bseProfilingTools.h"

#include "MazeBSE.h"

#define ENABLE_COLLISION

extern PhysicsDebugDraw gDebugDraw;

const bse::Real AI_TIME_STEP = (1.0f / 60.0f);
const bse::Real PHX_TIME_STEP = AI_TIME_STEP;
bse::UInt gCharactersDebugDraw = Character::cdb_path; //Character::cdb_all;

bse::Real gNumSearchesPerSec = 0.0f;

const bse::phx::BroadPhaseType g_broadPhaseType = bse::phx::BSE_BROADPHASE_SWEEPANDPRUNE;


//---------------------------------------------------------------------------------------------------------------------
bse::Vec2* getNodePos(const bse::ai::GraphNode* node)
{
  Maze::NodeState* state = (Maze::NodeState*)(node->getNodeData()->getData());
  return &(state->pos);
}


ManhattanDistanceHeuristic gGraphHeuristic;
//EulerDistanceHeuristic gGraphHeuristic;

//---------------------------------------------------------------------------------------------------------------------
Character::Character(Maze* maze, const CharacterDesc& desc) :
  m_position(0,0),
  m_maze(maze),
  m_shape(0),
  m_body(0),
  m_desc(desc)
{
  bse::phx::Scene* phScene = m_maze->getPhysicsScene();

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.position = desc.position;
  bodyDesc.mass = 1;
  bodyDesc.inertia = 0.1f;

  bse::phx::CircleDesc circleDesc;
  circleDesc.radius = desc.size/2.0f;
  bodyDesc.shapesDescs.push_back(&circleDesc);

  // create a physical body for the character. the position of the body is set from the descriptor
  m_body = phScene->createBody(&bodyDesc);
  bse::phx::ShapesList &shapes = m_body->getShapes();
  m_shape = (bse::phx::Circle*)shapes[0];

  physicsToAI();
}

//---------------------------------------------------------------------------------------------------------------------
Character::~Character()
{
  bse::phx::Scene* phScene = m_maze->getPhysicsScene();
  phScene->releaseBody(m_body);

  m_body = 0;
  m_shape = 0;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  \brief Keep the character position consistent with the physical body position
 */
void Character::physicsToAI()
{
  setPosition(m_body->getPosition());
}

//---------------------------------------------------------------------------------------------------------------------
void Character::render(bse::UInt flags)
{
  const bse::Real sizeCircle = m_shape->getRadius();
  Color charCol(0,1,0);
  if (flags & cdb_playercharacter)
    charCol = Color(0,1,1);
  gDebugDraw.drawCircle(m_position, sizeCircle, 12, charCol);
  if (flags & cdb_doublecircle)
    gDebugDraw.drawCircle(m_position, sizeCircle*0.25f, 12, charCol);

  if (flags & cdb_path)
  {
    Color pathCol(1,0,0);
    for (bse::ai::GraphPath::const_iterator iter = m_currentPath.begin(); iter != m_currentPath.end(); ++iter)
    {
      const bse::ai::GraphConnection* conn = (*iter);
      const bse::ai::GraphNode* source = conn->getSource();
      const bse::ai::GraphNode* dest = conn->getDest();

      bse::Vec2* posSource = ::getNodePos(source);
      bse::Vec2* posDest   = ::getNodePos(dest);

      gDebugDraw.drawLine(*posSource, *posDest, pathCol);
    }
  }

  if (flags & cdb_closestcell)
  {
    // inefficient, but good for debug draw.
    bse::ai::GraphNode* closestNode = m_maze->matchCharacterOnGraph(this);
    if (closestNode)
    {
      Color closestCol(1,1,1);
      bse::Vec2 closestNodePos = m_maze->getNodePos(closestNode);
      gDebugDraw.drawCircle(closestNodePos, 0.05f, 12, closestCol);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::initPath(bse::ai::GraphNode* initialTarget)
{
  m_maze->initPathToTarget(m_maze->matchCharacterOnGraph(this), initialTarget, m_currentPath);
}

//---------------------------------------------------------------------------------------------------------------------
bool AICharacter::update(bse::Real dt)
{
  bool runSearch = false;
  if (m_newTargetNodeSet && m_targetNode)
  {
    runSearch = true;
    m_maze->computePathToTarget(m_maze->matchCharacterOnGraph(this), m_targetNode, m_currentPath);
    m_newTargetNodeSet = false;
  } else if (!m_targetNode)
  {
    // just reset the path...
    m_currentPath.clear();
  }

  // easy... If I update the shortest path every frame (stress test)
  // I can just follow the route along the first connection of the path
  if (m_currentPath.size()>0)
  {
    const bse::ai::GraphConnection* firstConn = m_currentPath.front();
    bse::Vec2 moveTo = m_maze->getNodePos(firstConn->getDest());
    bse::Vec2 curPos = getPosition();
    bse::Vec2 dv = moveTo-curPos;
    bse::Real dist = dv.mag();
    dv.normalize();

    bse::Real deltaMove = speed * dt;
    bse::Vec2 newPos;
    bool moveOn = false;
    if (deltaMove>dist)
    {
      newPos = moveTo;
      moveOn = true;
    }
    else
    {
      newPos = curPos + dv*deltaMove;
    }

    m_body->setPosition(newPos);

    // time to go on?
    if (moveOn)
    {
      m_currentPath.pop_front();
      if (m_currentPath.empty())
      {
        runSearch = true;

        // no path to follow anymore
        // compute it again, getting the best one (time limited)
        m_maze->computePathToTarget(m_maze->matchCharacterOnGraph(this), m_targetNode, m_currentPath, true);
      }
    }
  }

  return runSearch;
}

//---------------------------------------------------------------------------------------------------------------------
bool AICharacter::updateStep1()
{
  bool runSearch = false;

  if (m_newTargetNodeSet && m_targetNode)
  {
    runSearch = true;
    m_newTargetNodeSet = false;
    m_maze->computePathToTargetAsynch(m_maze->matchCharacterOnGraph(this), m_targetNode, m_currentPath);
  } else if (!m_targetNode)
  {
    // just reset the path...
    m_currentPath.clear();
  }

  return runSearch;
}

//---------------------------------------------------------------------------------------------------------------------
bool AICharacter::updateStep2(bse::Real dt)
{
  bool runSearch = false;

  // easy... If I update the shortest path every frame (stress test)
  // I can just follow the route along the first connection of the path
  if (m_currentPath.size()>0)
  {
    const bse::ai::GraphConnection* firstConn = m_currentPath.front();
    bse::Vec2 moveTo = m_maze->getNodePos(firstConn->getDest());
    bse::Vec2 curPos = getPosition();
    bse::Vec2 dv = moveTo-curPos;
    bse::Real dist = dv.mag();
    dv.normalize();

    bse::Real deltaMove = speed * dt;
    bse::Vec2 newPos;
    bool moveOn = false;
    if (deltaMove>dist)
    {
      newPos = moveTo;
      moveOn = true;
    }
    else
    {
      newPos = curPos + dv*deltaMove;
    }

    m_body->setPosition(newPos);

    // time to go on?
    if (moveOn)
    {
      m_currentPath.pop_front();
      if (m_currentPath.empty())
      {
        // no path to follow anymore
        // compute it again, getting the best one (time limited)
        runSearch = true;
        m_maze->computePathToTargetAsynch(m_maze->matchCharacterOnGraph(this), m_targetNode, m_currentPath, true);
      }
    }
  }

  return runSearch;
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::setTarget(const bse::Vec2& target)
{
  // TODO
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::setTarget(Character* target)
{
  setTarget(m_maze->matchCharacterOnGraph(target));
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::setTarget(bse::ai::GraphNode* target)
{
  onNewTargetNodeSet(target);
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::onNewTargetNodeSet(bse::ai::GraphNode* target)
{
  m_newTargetNodeSet = true;
  m_targetNode = target;
}

//---------------------------------------------------------------------------------------------------------------------
void AICharacter::reset()
{
}

//---------------------------------------------------------------------------------------------------------------------
void PlayerCharacter::moveTargetPosition(const bse::Vec2& pos)
{
  m_targetPos = pos;
}

//---------------------------------------------------------------------------------------------------------------------
void PlayerCharacter::moveToTargetPosition()
{
  bse::Vec2 pos = m_targetPos;

  // move it, using newton law and reversing euler integrator
  // p(t+dt) = p(t) + v(t)*dt
  // v(t+dt) = v(t) + a(t)*dt
  // f = m*a

  bse::Vec2 oldPos = getPosition();
  bse::Vec2 vel = m_body->getLinearVelocity();
  bse::Vec2 velToTarget = (pos-oldPos)*(1.0f/PHX_TIME_STEP);
  bse::Vec2 accel = (velToTarget - vel) * (1.0f/PHX_TIME_STEP);
  m_body->addForce(accel*m_body->getMass());
}

//---------------------------------------------------------------------------------------------------------------------
bool PlayerCharacter::update(bse::Real dt)
{
  // move toward the target position
  moveToTargetPosition();
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void PlayerCharacter::render(bse::UInt flags)
{
  Character::render(flags);
  gDebugDraw.drawCircle(m_targetPos, 0.005f, 12, Color(1,0,0));
  gDebugDraw.drawCircle(m_targetPos, 0.1f, 12, Color(1,0,0));
}

//---------------------------------------------------------------------------------------------------------------------
void PlayerCharacter::reset()
{

}

//---------------------------------------------------------------------------------------------------------------------
Maze::Maze(const MazeDesc& desc, bse::TaskScheduler* taskScheduler) :
  m_desc(desc),
  m_mainPlayer(0),
  m_asynchPathFinder(false),
  m_taskScheduler(0)
{
  bse::ai::AISceneDesc aiSceneDesc;
  aiSceneDesc.numContexts = desc.numLogicalProcessors>0 ? desc.numLogicalProcessors : 1;
  aiSceneDesc.maxNumGraphNodes = m_desc.numberOfCols * m_desc.numberOfRows;
  m_aiScene = bse::ai::AIScene::create(&aiSceneDesc);

  bse::phx::SceneDesc phxSceneDesc;
  phxSceneDesc.broadPhaseType = g_broadPhaseType;

  m_physicsScene = bse::phx::Scene::create(&phxSceneDesc);
  initPathFinderGraph(desc.numberOfRows, desc.numberOfCols, desc.cellSize);
  initWallsCollisionShapes();
  initBox();

  if (m_desc.createPlayerCharacter)
  {
    m_mainPlayer = createPlayerCharacter(m_desc.playerCharacterPos, m_desc.playerCharacterSize);
  }

  // generate random ai players
  for (bse::UInt iPlayer=0; iPlayer<m_desc.numberOfAICharacters; ++iPlayer)
  {
    bse::Vec2 pos = getNodePos(m_mazeGraph->getNode(bseRandom(1, m_mazeGraph->getNumberOfNodes()-1)));

    AICharacter* aiChar = createNonPlayerCharacter(pos, m_desc.aiCharactersSize);
    aiChar->initPath(m_mainPlayer ? matchCharacterOnGraph(m_mainPlayer) : 0);
  }

  m_asynchPathFinder = taskScheduler != 0;
  m_taskScheduler = taskScheduler;
}

//---------------------------------------------------------------------------------------------------------------------
Maze::~Maze()
{
  destroyPathFinderGraph();
  while (!m_npcharacters.empty())
  {
    delete m_npcharacters.back();
    m_npcharacters.pop_back();
  }

  while (!m_players.empty())
  {
    delete m_players.back();
    m_players.pop_back();
  }

  bse::phx::Scene::destroy(m_physicsScene);
  bse::ai::AIScene::destroy(m_aiScene);
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::setAsynchPathFinder(bool enable)
{
  // Allow the change of this setting only if the task scheduler was initialized properly.
  if (m_taskScheduler)
  {
    m_asynchPathFinder = enable;
    reset(m_taskScheduler);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::reset(bse::TaskScheduler* taskScheduler)
{
  m_taskScheduler = taskScheduler;

  // Destroy characters and players. They will be re-created later.
  size_t numNPCharacter = m_npcharacters.size();
  bse::Vec2* aiInitPoses = new bse::Vec2[numNPCharacter];
  int iCharacter = 0;
  while (!m_npcharacters.empty())
  {
    aiInitPoses[iCharacter] = m_npcharacters.back()->getInitialPosition();
    ++iCharacter;
    delete m_npcharacters.back();
    m_npcharacters.pop_back();
  }

  int iPlayer = 0;
  size_t numPlayers = m_players.size();
  bse::Vec2* pInitPoses = new bse::Vec2[numPlayers];
  while (!m_players.empty())
  {
    pInitPoses[iPlayer] = m_players.back()->getInitialPosition();
    ++iPlayer;
    delete m_players.back();
    m_players.pop_back();
  }

  // Destroy and re-create the physics scene. This is not the most efficient solution but it's easier for the moment to just leave
  // the pathfinder graph (the random part) in place and rebuild the physics setup on top of it.
  bse::phx::Scene::destroy(m_physicsScene);
  bse::phx::SceneDesc phxSceneDesc;
  phxSceneDesc.broadPhaseType = g_broadPhaseType;
  m_physicsScene = bse::phx::Scene::create(&phxSceneDesc);
  initWallsCollisionShapes();
  initBox();

  bse::UInt numContexts = 1;
  if (m_taskScheduler && isAsynchPathFinderEnabled())
  {
    numContexts = m_taskScheduler->getNumWorkerThreads() + 1;
  }
  m_aiScene->reset(numContexts);

  if (m_desc.createPlayerCharacter)
  {
    m_mainPlayer = createPlayerCharacter(m_desc.playerCharacterPos, m_desc.playerCharacterSize);
  }

  // Re-create the ai characters
  for (bse::UInt iCharacter=0; iCharacter<m_desc.numberOfAICharacters; ++iCharacter)
  {
    const bse::Vec2& pos = aiInitPoses[iCharacter];
    AICharacter* aiChar = createNonPlayerCharacter(pos, m_desc.aiCharactersSize);
    aiChar->initPath(m_mainPlayer ? matchCharacterOnGraph(m_mainPlayer) : 0);
  }

  delete [] aiInitPoses;
  delete [] pInitPoses;
}

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2 Maze::getNodePos(bse::ai::GraphNode* node)
{
  return *(::getNodePos(node));
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::movePlayerTo(const bse::Vec2& pos)
{
  if (m_mainPlayer)
  {
    bse::Vec2 cPos(pos);
    clampPos(cPos);
    m_mainPlayer->moveTargetPosition(cPos);
  }
}

//---------------------------------------------------------------------------------------------------------------------
bse::ai::GraphNode* Maze::matchCharacterOnGraph(Character* character)
{
  // knowing that the graph in this example is a grid
  // we can get the closest node with some simple math
  // NB: the graph extends from (0,0) to positive x,y!!!
  bse::Vec2 pos = character->getPosition();

  // retrieve the area in which the character resides
  bse::UInt nCellsX = (bse::UInt) (pos.x / m_desc.cellSize);
  bse::UInt nCellsY = (bse::UInt) (pos.y / m_desc.cellSize);

  // candidate nodes indexes (the for nodes surrounding)
  unsigned int candidateIds[4] =
  {
    nCellsY * (m_desc.numberOfCols) + nCellsX,
    nCellsY * (m_desc.numberOfCols) + nCellsX + 1,
    nCellsY<m_desc.numberOfRows-1 ? (nCellsY+1) * (m_desc.numberOfCols) + nCellsX + 1 : -1,
    nCellsY<m_desc.numberOfRows-1 ? (nCellsY+1) * (m_desc.numberOfCols) + nCellsX : -1
  };

  bse::ai::GraphNode* candidateNodes[4] = {0,0,0,0};
  for (int i=0; i<4; ++i)
    if (candidateIds[i]!=-1 && candidateIds[i]<m_mazeGraph->getNumberOfNodes())
      candidateNodes[i] = m_mazeGraph->getNode(candidateIds[i]);

  bse::ai::GraphNode* minNode = 0;
  bse::Real minDist = 0;

  for (int i=0; i<4; ++i)
  {
    if (candidateNodes[i]==0)
      continue;

    if (minNode==0)
    {
      // it enters here the first time it finds a good node
      minNode = candidateNodes[i];
      minDist = (pos - getNodePos(candidateNodes[i])).sqrmag();
      continue;
    }

    bse::Real currDist = (pos - getNodePos(candidateNodes[i])).sqrmag();
    if (currDist < minDist)
    {
      minDist = currDist;
      minNode = candidateNodes[i];
    }
  }

  return minNode;
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::initPathToTarget(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget)
{
  bse::ai::SearchParameters searchParams;
  searchParams.searchType = bse::ai::SearchParameters::SEARCH_TYPE_COMPLETE;
  m_mazeGraph->computeShortestPath(source, target, pathToTarget, searchParams);
}

//---------------------------------------------------------------------------------------------------------------------
bool Maze::computePathToTarget(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget, bool bestPathOnFail)
{
  bse::ai::SearchParameters searchParams;
  searchParams.searchType = bse::ai::SearchParameters::SEARCH_TYPE_TIME_LIMITED;
  if (bestPathOnFail)
  {
    searchParams.resultType = bse::ai::SearchParameters::RESULT_TYPE_PATH_TO_BEST_NODE_ONFAIL;
  }
  else
  {
    searchParams.resultType = bse::ai::SearchParameters::RESULT_TYPE_NOACTION_ONFAIL;
  }
  searchParams.params.timeLimit = .5f; // msecs
  bool searchResult = m_mazeGraph->computeShortestPath(source, target, pathToTarget, searchParams);
  (void)searchResult;

  return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool Maze::computePathToTargetAsynch(bse::ai::GraphNode* source, bse::ai::GraphNode* target, bse::ai::GraphPath& pathToTarget, bool bestPathOnFail)
{
  BSE_ASSERT(m_taskScheduler);
  bse::ai::SearchParameters searchParams;
  searchParams.searchType = bse::ai::SearchParameters::SEARCH_TYPE_TIME_LIMITED;
  if (bestPathOnFail)
  {
    searchParams.resultType = bse::ai::SearchParameters::RESULT_TYPE_PATH_TO_BEST_NODE_ONFAIL;
  }
  else
  {
    searchParams.resultType = bse::ai::SearchParameters::RESULT_TYPE_NOACTION_ONFAIL;
  }
  searchParams.params.timeLimit = .5f; // msecs

  UpdatePathTask *updateTask = m_updatePathTasks.getObject();
  updateTask->setup(this, source, target, &pathToTarget, bestPathOnFail);
  m_taskScheduler->scheduleTask(updateTask);
  return true;
}



//---------------------------------------------------------------------------------------------------------------------
/**
 *  \brief Main game update loop
 *  This the order:
 *  0) get current status from physics
 *  1) update the ai world.
 *  2) update the physics world
 *
 */
void Maze::updateGame()
{
  m_numSearches = 0;

  // transfer data from physics to ai (todo: this will be done using dynamics agents, wip)
  for (std::vector<PlayerCharacter*>::iterator charIter = m_players.begin(); charIter != m_players.end(); ++charIter)
    (*charIter)->physicsToAI();

  for (std::vector<AICharacter*>::iterator charIter = m_npcharacters.begin(); charIter != m_npcharacters.end(); ++charIter)
    (*charIter)->physicsToAI();


  for (size_t iChar = 0; iChar < m_npcharacters.size(); ++iChar)
  {
    m_npcharacters[iChar]->setTarget(getMainPlayer());
  }

  // update players
  for (std::vector<PlayerCharacter*>::iterator charIter = m_players.begin(); charIter != m_players.end(); ++charIter)
  {
    (*charIter)->update(AI_TIME_STEP);
  }

  if(isAsynchPathFinderEnabled())
  {
    // Enabling the asynchronous path finder should not be possible if the task scheduler hasn't been initialized.
    BSE_ASSERT(m_taskScheduler);

    // Perform the update in two steps.
    // 1) check if the characters have a new target, if so, queue a task to update the path. Then give time to complete.
    for (std::vector<AICharacter*>::iterator charIter = m_npcharacters.begin(); charIter != m_npcharacters.end(); ++charIter)
    {
      if ((*charIter)->updateStep1())
      {
        ++m_numSearches;
      }
    }

    m_taskScheduler->waitForAllTasks();

    // 2) some characters might need a recomputation of the path.
    for (std::vector<AICharacter*>::iterator charIter = m_npcharacters.begin(); charIter != m_npcharacters.end(); ++charIter)
    {
      if ((*charIter)->updateStep2(AI_TIME_STEP))
      {
        ++m_numSearches;
      }
    }

    m_taskScheduler->waitForAllTasks();

    // Relese frame objects.
    m_updatePathTasks.clear();
  }
  else
  {
    // update non-players
    for (std::vector<AICharacter*>::iterator charIter = m_npcharacters.begin(); charIter != m_npcharacters.end(); ++charIter)
    {
      if ((*charIter)->update(AI_TIME_STEP))
      {
        ++m_numSearches;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::updatePhysics()
{
  m_physicsScene->simulate(PHX_TIME_STEP);
}

//---------------------------------------------------------------------------------------------------------------------
PlayerCharacter* Maze::createPlayerCharacter(const bse::Vec2& pos, bse::Real size)
{
  CharacterDesc desc;
  desc.position = pos;
  desc.size = size;
  PlayerCharacter* character = new PlayerCharacter(this, desc);
  m_players.push_back(character);
  return character;
}

//---------------------------------------------------------------------------------------------------------------------
AICharacter* Maze::createNonPlayerCharacter(const bse::Vec2& pos, bse::Real size)
{
  CharacterDesc desc;
  desc.position = pos;
  desc.size = size;
  AICharacter* character = new AICharacter(this, desc);
  m_npcharacters.push_back(character);
  return character;
}

//---------------------------------------------------------------------------------------------------------------------
int Maze::getNodeState(bse::ai::GraphNode* node)
{
  NodeState* nodeState = (NodeState*)(node->getNodeData()->getData());
  return nodeState->state;
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::setNodeState(bse::ai::GraphNode* node, int state)
{
  NodeState* nodeState = (NodeState*)(node->getNodeData()->getData());
  nodeState->state = state;
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::getNodeNeighbours(bse::ai::GraphNode* node, int* nbs)
{
  NodeState* ns = (NodeState*)(node->getNodeData()->getData());
  int numCols = m_desc.numberOfCols;
  int numRows = m_desc.numberOfRows;

  int ln = -1, rn = -1, un = -1, dn = -1; // neighbours (spatially)

  if (ns->col < numCols-1) // right neighbour exists
    rn = ns->id + 1;
  if (ns->col > 0)          // left neighbour exists
    ln = ns->id - 1;
  if (ns->row < numRows-1) // up neighbour exists
    un = (ns->row+1) * numCols + ns->col;
  if (ns->row > 0)          // down neighbour exists
    dn = (ns->row-1) * numCols + ns->col;

  nbs[0] = rn;
  nbs[1] = ln;
  nbs[2] = un;
  nbs[3] = dn;
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::setNodeNeighboursToFrontier(bse::ai::GraphNode* node, std::vector<bse::ai::GraphNode*>& frontier)
{
  int nbs[4];
  getNodeNeighbours(node, nbs);

  for (int i=0; i<4; ++i)
    if (nbs[i]!=-1)
    {
      bse::ai::GraphNode* nbNode = m_mazeGraph->getNode(nbs[i]);
      int nbState = getNodeState(nbNode);
      if (nbState == NodeState::NODE_OUT)
      {
        setNodeState(nbNode, NodeState::NODE_FRONTIER);
        frontier.push_back(nbNode);
      }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::initPathFinderGraph(bse::UInt numRows, bse::UInt numCols, bse::Real cellSize)
{
  int i=0, row=0, col=0;

  // force density to 1, the grave is carved through prim's algorithm
  const int NUM_ROWS = numRows;
  const int NUM_COLS = numCols;
  const int NUM_NODES = NUM_ROWS * NUM_COLS;
  const bse::Real CELL_SIZE = cellSize; //0.25f;

  const bse::Real horizontalCost = 1;

  m_nodeData = new bse::ai::GraphNodeData[NUM_NODES];

  // temp struct used to create the maze
  NodeState* nodesState = new NodeState[NUM_NODES];
  m_nodeStates = nodesState;

  bse::Real x = 0, y = 0;
  for (row=0; row<NUM_ROWS; ++row)
  {
    x = 0;
    for (col=0; col<NUM_COLS; ++col)
    {
      int id = row * NUM_COLS + col;

      // initialize the helper state
      nodesState[id].state = NodeState::NODE_OUT;
      nodesState[id].row = row;
      nodesState[id].col = col;
      nodesState[id].id = id;

      // initialize position on the grid
      //m_nodePos[id] = bse::Vec2(x,y);
      nodesState[id].pos = bse::Vec2(x,y);
      x += CELL_SIZE;
    }

    y += CELL_SIZE;
  }

  // temporary set data to an helper structure for maze creation
  for (i=0; i<NUM_NODES; ++i)
  {
    m_nodeData[i].setData(&nodesState[i]);
  }


  // create the pathfinder graph
  bse::ai::Graph* graph = getAIScene()->createPathFinderGraph(&gGraphHeuristic);

  // store the graph pointer
  m_mazeGraph = graph;

  // temporary
  bse::ai::GraphNode** nodes = new bse::ai::GraphNode*[NUM_NODES];

  for (i=0; i<NUM_NODES; ++i)
  {
    nodes[i] = graph->createNode(&m_nodeData[i]);
  }


  // use prim's algorithm to create a perfect maze
      // pick a random initial node
      std::vector<bse::ai::GraphNode*> frontier;
      bse::ai::GraphNode* node = graph->getNode(bseRandom(0, NUM_NODES));
      setNodeState(node, NodeState::NODE_IN);
      setNodeNeighboursToFrontier(node, frontier);

      while (!frontier.empty())
      {
        // pick a frontier randomly
        bse::UInt pickOneId = bseRandom(0, (bse::UInt)frontier.size()-1);
        bse::ai::GraphNode* pickOne = frontier[pickOneId];
        // remove it
        if (pickOneId<frontier.size()-1)
          frontier[pickOneId] = frontier.back();
        frontier.pop_back();

        // choose one of its neighbours that is alredy IN and create a bidirectional connection
        int nbs[4];
        getNodeNeighbours(pickOne, nbs);

        // TODO: make this a bit more random
        while (true)
        {
          int i = bseRandom(0,4);
          if (i<4 && nbs[i]!=-1)
          {
            bse::ai::GraphNode* neighFrom = graph->getNode(nbs[i]);
            if (getNodeState(neighFrom) == NodeState::NODE_IN)
            {
              // connect...
              graph->createConnection(pickOne, neighFrom, horizontalCost);
              graph->createConnection(neighFrom, pickOne, horizontalCost);

              break; // i'm done...
            }
          }
        }

        setNodeState(pickOne, NodeState::NODE_IN);
        setNodeNeighboursToFrontier(pickOne, frontier);


      }

  // clean up un needed stuff
  delete [] nodes;
}

//---------------------------------------------------------------------------------------------------------------------
// x, y is the lower left corner
void createRoom(bse::phx::Scene* scene, bse::Real x, bse::Real y, bse::Real sizeX, bse::Real sizeY, bse::Real wallsThick)
{
  bse::phx::Scene* gScene = scene;

  bse::phx::BoxDesc x_boxDesc;
  x_boxDesc.dims.x = sizeX;
  x_boxDesc.dims.y = wallsThick;

  bse::phx::BoxDesc y_boxDesc;
  y_boxDesc.dims.x = wallsThick;
  y_boxDesc.dims.y = sizeY;

  bse::phx::BodyDesc bodyDesc;
  bodyDesc.flags |= BSE_BODYFLAG_STATIC;

  bse::phx::Body* body;

  // lower bound
  bodyDesc.shapesDescs.push_back(&x_boxDesc);
  body = gScene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x + sizeX / 2.0f, y-0.05f);

  // higher bound
  bodyDesc.shapesDescs.push_back(&x_boxDesc);
  body = gScene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x + sizeX / 2.0f, y+0.05f + sizeY);

  // left bound
  bodyDesc.shapesDescs.push_back(&y_boxDesc);
  body = gScene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x-0.05f, y + sizeY/2.0f);

  // right bound
  bodyDesc.shapesDescs.push_back(&y_boxDesc);
  body = gScene->createBody(&bodyDesc);
  bodyDesc.shapesDescs.clear();
  body->setPosition(x+0.05f + sizeX, y + sizeY/2.0f);
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::initBox()
{
  const bse::Real thickness = 0.05f;

  bse::Real height = m_desc.cellSize * m_desc.numberOfRows-thickness;
  bse::Real width  = m_desc.cellSize * m_desc.numberOfCols-thickness;
  bse::Real x = -2*thickness;
  bse::Real y = -2*thickness;

  createRoom(m_physicsScene, x, y, width, height, thickness);
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::initWallsCollisionShapes()
{
  const int numNodes = m_mazeGraph->getNumberOfNodes();

  static int nWalls = 0;
  static int nWallsNoFilter = 0;

  const bse::Real thickness = 0.05f;

  // walls markers to avoid duplicate creation
  bool* walls = new bool[numNodes];
  for (int i=0; i<numNodes; ++i)
    walls[i] = false;

  for (int iNode=0; iNode<numNodes; ++iNode)
  {
    bse::ai::GraphNode* node = m_mazeGraph->getNode(iNode);

    // retrieve node position
    bse::Vec2* pos = ::getNodePos(node);

    int nbs[4];
    getNodeNeighbours(node, nbs);

    bse::Real cellSize  = m_desc.cellSize;
    bse::Real cellSize2 = m_desc.cellSize/2.0f;

    if (nbs[0] != -1 && !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[0])))
    {
      ++nWallsNoFilter;
      NodeState* ns = &m_nodeStates[iNode];
      unsigned int wi = ns->col*(m_desc.numberOfRows)+ns->row;
      if (!walls[wi])
      {
        ++nWalls;
        walls[wi] = true;
        bse::phx::BoxDesc boxDesc;
        bse::phx::BodyDesc     bodyDesc;
        bodyDesc.flags |= BSE_BODYFLAG_STATIC;
        bodyDesc.position = bse::Vec2(pos->x + cellSize2, pos->y);
        boxDesc.dims = bse::Vec2(thickness, cellSize);
        bodyDesc.shapesDescs.push_back(&boxDesc);
        m_physicsScene->createBody(&bodyDesc);
      }
    }

    if (nbs[1] != -1 && !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[1])))
    {
      ++nWallsNoFilter;
      NodeState* ns = &m_nodeStates[nbs[1]];
      unsigned int wi = ns->col*(m_desc.numberOfRows)+ns->row;
      if (!walls[wi])
      {
        ++nWalls;
        walls[wi] = true;
        bse::phx::BoxDesc boxDesc;
        bse::phx::BodyDesc     bodyDesc;
        bodyDesc.flags |= BSE_BODYFLAG_STATIC;
        bodyDesc.position = bse::Vec2(pos->x - cellSize2, pos->y);
        boxDesc.dims = bse::Vec2(thickness, cellSize);
        bodyDesc.shapesDescs.push_back(&boxDesc);
        m_physicsScene->createBody(&bodyDesc);
      }
    }
  }


  // horizontal walls
  for (int i=0; i<numNodes; ++i)
  {
    walls[i] = false;
  }

  for (int iNode=0; iNode<numNodes; ++iNode)
  {
    bse::ai::GraphNode* node = m_mazeGraph->getNode(iNode);

    // retrieve node position
    bse::Vec2* pos = ::getNodePos(node);

    int nbs[4];
    getNodeNeighbours(node, nbs);

    bse::Real cellSize  = m_desc.cellSize;
    bse::Real cellSize2 = m_desc.cellSize/2.0f;

    if (nbs[2] != -1 && !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[2])))
    {
      ++nWallsNoFilter;
      NodeState* ns = &m_nodeStates[iNode];
      unsigned int wi = ns->col*(m_desc.numberOfRows)+ns->row;
      if (!walls[wi])
      {
        ++nWalls;
        walls[wi] = true;
        bse::phx::BoxDesc boxDesc;
        bse::phx::BodyDesc     bodyDesc;
        bodyDesc.flags |= BSE_BODYFLAG_STATIC;
        bodyDesc.position = bse::Vec2(pos->x, pos->y + cellSize2);
        boxDesc.dims = bse::Vec2(cellSize, thickness);
        bodyDesc.shapesDescs.push_back(&boxDesc);
        m_physicsScene->createBody(&bodyDesc);
      }
    }

    if (nbs[3] != -1 && !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[3])))
    {
      ++nWallsNoFilter;
      NodeState* ns = &m_nodeStates[nbs[3]];
      unsigned int wi = ns->col*(m_desc.numberOfRows)+ns->row;
      if (!walls[wi])
      {
        ++nWalls;
        walls[wi] = true;
        bse::phx::BoxDesc boxDesc;
        bse::phx::BodyDesc     bodyDesc;
        bodyDesc.flags |= BSE_BODYFLAG_STATIC;
        bodyDesc.position = bse::Vec2(pos->x, pos->y - cellSize2);
        boxDesc.dims = bse::Vec2(cellSize, thickness);
        bodyDesc.shapesDescs.push_back(&boxDesc);
        m_physicsScene->createBody(&bodyDesc);
      }
    }
  }

  // clean up
  delete [] walls;
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::destroyPathFinderGraph()
{
  delete [] m_nodeData;
  delete [] m_nodeStates;

  m_nodeData = 0;
  m_nodeStates = 0;

  getAIScene()->destroyPathFinderGraph(m_mazeGraph);
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::renderPathFinderGraph(bse::UInt debugDraw)
{
  if (!m_mazeGraph || debugDraw == Maze::mdb_none)
    return;

  int numNodes = m_mazeGraph->getNumberOfNodes();
  int numConn = m_mazeGraph->getNumberOfConnections();

  Color graphCol(0.5f,0.5f,0.5f);

  // draw all the nodes
  if (debugDraw & Maze::mdb_intersections)
    for (int iNode=0; iNode<numNodes; ++iNode)
    {
      bse::ai::GraphNode* node = m_mazeGraph->getNode(iNode);

      // retrieve node position
      bse::Vec2* pos = ::getNodePos(node);

      gDebugDraw.drawCircle(*pos, 0.05f, 12, graphCol);
    }

  // draw all the connections
  if (debugDraw & Maze::mdb_connections)
    for (int iConn=0; iConn<numConn; ++iConn)
    {
      const bse::ai::GraphConnection* conn = m_mazeGraph->getConnection(iConn);
      bse::ai::GraphNode* source = conn->getSource();
      bse::ai::GraphNode* dest = conn->getDest();

      bse::Vec2* posSource = ::getNodePos(source);
      bse::Vec2* posDest   = ::getNodePos(dest);

      gDebugDraw.drawLine(*posSource, *posDest, graphCol); // TODO include thickness to highlight the cost: conn->getCost());
      // draw a circle in the source
    }

  // draw all the walls
  // TODO: THIS IS SOOOO SLOWWWWWWWWWW
  if (debugDraw & Maze::mdb_walls)
  {
    Color wallCol(1,1,1);
    for (int iNode=0; iNode<numNodes; ++iNode)
    {
      bse::ai::GraphNode* node = m_mazeGraph->getNode(iNode);
        // retrieve node position
      bse::Vec2* pos = ::getNodePos(node);

      int nbs[4];
      getNodeNeighbours(node, nbs);

      bse::Real cellSize2 = m_desc.cellSize/2.0f;

      // borders
      if (nbs[0] == -1 || !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[0])))
        gDebugDraw.drawLine(bse::Vec2(pos->x + cellSize2, pos->y - cellSize2), bse::Vec2(pos->x + cellSize2, pos->y + cellSize2), wallCol);
      if (nbs[1] == -1 || !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[1])))
        gDebugDraw.drawLine(bse::Vec2(pos->x - cellSize2, pos->y - cellSize2), bse::Vec2(pos->x - cellSize2, pos->y + cellSize2), wallCol);
      if (nbs[2] == -1 || !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[2])))
        gDebugDraw.drawLine(bse::Vec2(pos->x - cellSize2, pos->y + cellSize2), bse::Vec2(pos->x + cellSize2, pos->y + cellSize2), wallCol);
      if (nbs[3] == -1 || !m_mazeGraph->nodesAreConnected(node, m_mazeGraph->getNode(nbs[3])))
        gDebugDraw.drawLine(bse::Vec2(pos->x - cellSize2, pos->y - cellSize2), bse::Vec2(pos->x + cellSize2, pos->y - cellSize2), wallCol);
    }
  }

  if (debugDraw & Maze::mdb_walls_collisionshapes)
  {
    Color wallCol(0.0f,1.0f,0.0f);
    bse::phx::ShapesList &shapes = m_physicsScene->getShapes();
    for(bse::phx::ShapesList::iterator shapesIter=shapes.begin(); shapesIter!=shapes.end(); ++shapesIter)
    {
      bse::phx::Shape* shape = (*shapesIter);
      if (shape->getBody()->isStatic())
      {
        gDebugDraw.drawShape(shape, wallCol, SHAPE_DRAW_FILLED);
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void Maze::renderCharacters()
{
  for (std::vector<PlayerCharacter*>::iterator charIter = m_players.begin(); charIter != m_players.end(); ++charIter)
    (*charIter)->render(gCharactersDebugDraw | Character::cdb_playercharacter | Character::cdb_doublecircle);

  for (std::vector<AICharacter*>::iterator charIter = m_npcharacters.begin(); charIter != m_npcharacters.end(); ++charIter)
    (*charIter)->render(gCharactersDebugDraw);
}


void UpdatePathTask::compute()
{
  m_lastSearchResult = m_maze->computePathToTarget(m_source, m_target, *m_pathToTarget, m_bestPathOnFail);
}

void UpdatePathTask::setup(
  Maze *maze,
  bse::ai::GraphNode* source,
  bse::ai::GraphNode* target,
  bse::ai::GraphPath* pathToTarget, bool bestPathOnFail)
{
  m_maze = maze;
  m_source = source;
  m_target = target;
  m_pathToTarget = pathToTarget;
  m_bestPathOnFail = bestPathOnFail;

  m_lastSearchResult = false;
}
