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

#include <vector>

#include <GLFW/glfw3.h>

// demo includes
#include "DrawUtils.h"
#include "bseProfilingTools.h"
#include "bseAIContext.h"
#include "MazeBSE.h"

#include "SysUtils.h"

// fw decl
void initGame();
void shutDownGame();
void shutDownGraphics();

PhysicsDebugDraw gDebugDraw;

bse::UInt gGraphDebugDraw      = Maze::mdb_walls | Maze::mdb_walls_collisionshapes;
Maze* gMaze = 0;
bse::TaskScheduler* gTaskScheduler = 0;

//---------------------------------------------------------------------------------------------------------------------
void increaseNumberOfWorkers()
{
  if (gTaskScheduler)
  {
    bse::UInt numWorkerThreads = gTaskScheduler->getNumWorkerThreads();
    ++numWorkerThreads;
    delete gTaskScheduler;
    gTaskScheduler = bse::initDefaultScheduler(numWorkerThreads, 0);
    gMaze->reset(gTaskScheduler);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void decreaseNumberOfWorkers()
{
  if (gTaskScheduler)
  {
    bse::UInt numWorkerThreads = gTaskScheduler->getNumWorkerThreads();
    --numWorkerThreads;
    bse::destroyDefaultScheduler(gTaskScheduler);
    gTaskScheduler = bse::initDefaultScheduler(numWorkerThreads > 0 ? numWorkerThreads : 1, 0);
    gMaze->reset(gTaskScheduler);
  }
}

float totalAITime = 0;
int numSteps = 0;

//---------------------------------------------------------------------------------------------------------------------
void keyboardCallback(int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
  {
    switch (key)
    {
    case GLFW_KEY_1:
      gMaze->setAsynchPathFinder(!gMaze->isAsynchPathFinderEnabled());
      totalAITime = 0;
      numSteps = 0;
    break;
    case GLFW_KEY_2:
      gMaze->reset(gTaskScheduler);
      totalAITime = 0;
      numSteps = 0;
      break;
    case GLFW_KEY_KP_ADD:
      increaseNumberOfWorkers();
      break;
    case GLFW_KEY_KP_SUBTRACT:
      decreaseNumberOfWorkers();
      break;
    case GLFW_KEY_SPACE:
      break;
    }
  }

  if (action == GLFW_PRESS || action == GLFW_REPEAT)
  {  
    PlayerCharacter* player = 0;
    bse::Vec2 pos(0,0);
    const bse::Real INC_POS = 0.025f;

    if (gMaze && gMaze->getMainPlayer())
    {
      player = gMaze->getMainPlayer();
      pos = player->getPosition();
    }

    switch (key)
    {
    case GLFW_KEY_LEFT:
      pos.x -= INC_POS;
      break;

    case GLFW_KEY_RIGHT:
      pos.x += INC_POS;
      break;

    case GLFW_KEY_UP:
      pos.y += INC_POS;
      break;

    case GLFW_KEY_DOWN:
      pos.y -= INC_POS;
      break;
    }

    // clamp pos
    gMaze->movePlayerTo(pos);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void frameCallback()
{
  if (gMaze)
  {
    gDebugDraw.clearAll();

    gMaze->renderPathFinderGraph(gGraphDebugDraw);
    gMaze->renderCharacters();


#ifdef BSE_ENABLE_PROFILER
    bse::ai::AIContextManager* contextManager = gMaze->getAIScene()->getContextManager();
    contextManager->clearProfilerData();
    bse::ProfilerTask* aiProfilerRootData = contextManager->getProfilerData();

    bse::ProfilerTask* physicsProfilerRootData = gMaze->getPhysicsScene()->getProfiler()->getRoot();
#endif

    bse::GenericTimer timer;

    timer.startTimer();
    gMaze->updateGame();
    float aiUpdateTime = timer.stopTimer();
	  totalAITime += aiUpdateTime;
	  ++numSteps;

    timer.startTimer();
    gMaze->updatePhysics();
    float physicsUpdateTime = timer.stopTimer();

    // Output step timer result.
    int base = 10, offset = 15;
    drawString(10, base, "AI Time: %.5f - %.5f", aiUpdateTime, totalAITime / numSteps);
    base += offset;
    drawString(10, base, "Physics Time: %.5f", physicsUpdateTime);
    base += offset;
    drawString(10, base, "Asynch Path Finder: %s", gMaze->isAsynchPathFinderEnabled() ? "On" : "Off");
    base += offset;
    if (gTaskScheduler && gMaze->isAsynchPathFinderEnabled())
    {
      drawString(10, base, "Num threads: %d", gTaskScheduler->getNumWorkerThreads() + 1);
      base += offset;
    }
    drawString(10, base, "Num Searches: %d", gMaze->getNumSearches());
    base += offset;

#ifdef BSE_ENABLE_PROFILER
    Color textCol(1,1,1);
    if (!gMaze->isAsynchPathFinderEnabled())
    {
      base = reportProfilerOutput(aiProfilerRootData, 10, base, 10, offset, textCol);
    }

    base += offset;
    base = reportProfilerOutput(physicsProfilerRootData, 10, base, 10, offset, textCol);

#endif

    drawString(10, base, "Use arrow keys to move.");
    base += offset;

    gDebugDraw.renderAll();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void mouseCallback( int button, int state, int x, int y )
{

}

//---------------------------------------------------------------------------------------------------------------------
void mouseMotionCallback( int x, int y )
{

}

//---------------------------------------------------------------------------------------------------------------------
void initGame()
{
  bse::UInt numLogicalProc = getNumLogicalProcessors();

  MazeDesc mazeDesc;
  mazeDesc.numLogicalProcessors = numLogicalProc;
  mazeDesc.numberOfCols = 25;
  mazeDesc.numberOfRows = 25;
  mazeDesc.cellSize = 0.25f;

  // players
  mazeDesc.createPlayerCharacter = true;
  mazeDesc.playerCharacterPos = bse::Vec2(0,0);
  mazeDesc.playerCharacterSize = 0.1f;

  mazeDesc.numberOfAICharacters = 16;
  mazeDesc.aiCharactersSize = 0.1f;

  // Spawn threads for parallel tasks. The main thread does some work while waiting so spawn one
  // thread less than the number of logical processors.
  bse::UInt numThreads = mazeDesc.numLogicalProcessors>1 ? mazeDesc.numLogicalProcessors-1 : 0;
  if (numThreads > 0)
  {
    gTaskScheduler = bse::initDefaultScheduler(numThreads, 0);
  }

  gMaze = new Maze(mazeDesc, gTaskScheduler);
}

//---------------------------------------------------------------------------------------------------------------------
void shutDownGame()
{
  delete gMaze;
  gMaze = 0;
  delete gTaskScheduler;
  gTaskScheduler = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void initGraphics(int argc, char** argv)
{
  // create a glut window with camera management.
  RenderSceneDesc renderSceneDesc;
  renderSceneDesc.argc = argc;
  renderSceneDesc.argv = argv;
  renderSceneDesc.windowTitle = "MazeBSE - SeeBeexSoft 2008-2011";
  renderSceneDesc.windowHeight = 768;
  renderSceneDesc.windowWidth = 1024;
  renderSceneDesc.windowStartX = 0;
  renderSceneDesc.windowStartY = 0;
  renderSceneDesc.updateMode = RENDER_UPDATEMODE_TIMED; // RENDER_UPDATEMODE_CONTINUOS; // RENDER_UPDATEMODE_MANUAL;
  renderSceneDesc.updateTiming = 1000 / 60; // ms
  // setup callbacks
  renderSceneDesc.keyboardFunc = keyboardCallback;
  renderSceneDesc.frameFunc = frameCallback; // paint
  renderSceneDesc.mouseFunc = mouseCallback;
  renderSceneDesc.mouseMotionFunc = mouseMotionCallback;
  renderSceneDesc.shutdownFunc = shutDownGame;

  // setup camera
  CameraDesc camera;
  camera.fixed = true; // needs to be fixed, otherwise arrows key would move both players and camera... confusing!
  camera.startZoom = 4;
  camera.zoomTick = 0.5f;
  camera.zoomMax = 100.0f;
  camera.zoomMin = 0.5f;
  camera.startX = 0.6f;
  camera.startY = -0.05f;
  camera.viewTick = 0.1f;

  renderSceneDesc.camera = camera;

  initDrawUtils(renderSceneDesc);
}

//---------------------------------------------------------------------------------------------------------------------
void shutDownGraphics()
{
  shutdownDrawUtils();
}

//---------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  srand( (unsigned)getCurrentTime() );

  initGame();
  initGraphics(argc, argv);

	return 0;
}
