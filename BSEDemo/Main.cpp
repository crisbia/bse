///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// BSEDemo
// Cristian Bianchi
// 2008-2011
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <vector>

// demo includes
#include "DrawUtils.h"
#include "BSEDemo.h"

// bse includes
#include "bseTools.h"
#include "bseProfilingTools.h"

// test framework refactoring
#include "TestFramework.h"
#include "TestUtils.h"

#include "SysUtils.h"

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>


class SimpleLogger : public BSEDemo::Logger
{
public:
  virtual void log(const char* msg)
  {
    printf("%s", msg);
  }
};

BSEDemo::InputControl gInputControl;
PhysicsDebugDraw gDebugDraw;
SimpleLogger gLogger;

///////////////////////////
bse::ToolsHandler*  gToolsHandler = 0;
bse::phx::Scene*    gScene = 0;

// Global parameters.
bodyDrawFlags   g_bodyFlags  = BODY_DRAW_ALL;
shapeDrawFlags  g_shapeFlags = SHAPE_DRAW_ALL;
bool g_bDrawContactPoints = false;

const bool g_bSingleStep = false;
bool g_bPaused = false;

//---------------------------------------------------------------------------------------------------------------------
// Contact points debug data
class DemoContactFeedback : public bse::phx::ContactFeedback
{
public:
  void onContactReport(const bse::phx::Contact* contact)
  {
    BSEDemo::ContactPointInfo cPoint;
    cPoint.position = contact->contactPoint;
    cPoint.normal = contact->contactNormal;
    contactPoints.push_back(cPoint);

    BSEDemo::gTestManager->handleContact(cPoint);
  }

  std::vector<BSEDemo::ContactPointInfo> contactPoints;

  void reset() { contactPoints.resize(0); }
  void renderContactPoints()
  {
    Color colRed(1,0,0), colGreen(0,1,0), colYellow(1,1,0);

    int numContact = (int)contactPoints.size();
    for (int i=0; i<numContact; ++i)
    {
      BSEDemo::ContactPointInfo contact = contactPoints[i];
      bse::Vec2 pos = contact.position;
      bse::Vec2 posEnd = pos + contact.normal * 0.1f;
      gDebugDraw.drawBox(pos, bse::Vec2(0.01f, 0.01f), colRed);
      gDebugDraw.drawLine(pos, posEnd, colGreen);
      gDebugDraw.drawCircle(posEnd, 0.01f, 6, colGreen);
    }
  }
};

DemoContactFeedback gContactFeedback;

//---------------------------------------------------------------------------------------------------------------------
void createDefaultScene()
{
  bse::phx::SceneDesc sceneDesc;
  sceneDesc.timeStep = 1.0f/60.0f;
  sceneDesc.solverIterations = 12;
  sceneDesc.gravity = bse::Vec2(0,-1);
  sceneDesc.broadPhaseType = bse::phx::BSE_BROADPHASE_SWEEPANDPRUNE;

  bse::phx::Scene* scene = gToolsHandler->createPhysicsScene(&sceneDesc);
  gScene = scene;

  gScene->setContactFeedback(&gContactFeedback);
}

void destroyDefaultScene()
{
  gToolsHandler->destroyPhysicsScene(gScene);
}

bse::Int framePeriod = 16;
bse::Int mainWindow;

//---------------------------------------------------------------------------------------------------------------------
// This is used to control the frame rate (60Hz).
void timerCallback(int)
{
  // glutSetWindow(mainWindow);
  // glutPostRedisplay();
  // glutTimerFunc(framePeriod, timerCallback, 0);
}

//---------------------------------------------------------------------------------------------------------------------
// Simple switch of the render flags.
void updateDrawFlags()
{
  if (g_bodyFlags == BODY_DRAW_ALL && g_shapeFlags == SHAPE_DRAW_ALL)
  {
    g_bodyFlags = BODY_DRAW_SHAPES;
    g_shapeFlags = SHAPE_DRAW_WIREFRAME;
  }
  else
  {
    g_bodyFlags = BODY_DRAW_ALL;
    g_shapeFlags = SHAPE_DRAW_ALL;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void updatePhysics(float timeStep)
{
  gContactFeedback.reset();
  BSEDemo::gTestManager->updateCurrentTest(timeStep, timeStep);
  gScene->simulate(timeStep);
}

//---------------------------------------------------------------------------------------------------------------------
// Window refresh callback. Takes care of both the simulation and the debug rendering.
void redraw()
{
  float timeStep = 1.0f / 60.0f;
  gDebugDraw.clearAll();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.75f, 0.75f, 0.75f, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  bse::GenericTimer drawTimer;
  drawTimer.startTimer();

  // Render the current test.
  BSEDemo::gTestManager->renderCurrentTest(timeStep, timeStep);

  gDebugDraw.drawBodies(gScene, g_bodyFlags, g_shapeFlags);

  if (g_bDrawContactPoints)
    gContactFeedback.renderContactPoints();

  // Draw all the debug information and clear the internal buffers.
  gDebugDraw.renderAll();

  const Color textCol(0.2f, 0.2f, 0.2f);
  const int offset = 15;
  int base = 15;

  float drawElapsed = drawTimer.stopTimer();
  drawString(5, base, textCol, "Draw time: %.5f", drawElapsed);
  base += offset;


  if (!g_bSingleStep && !g_bPaused)
  {
    gContactFeedback.reset();

    static float totalTime = 0;
    static int numSteps = 1;

    bse::GenericTimer stepTimer;
    stepTimer.startTimer();
     // Update the current test (the manager takes care of the fact that there's an active one).
    BSEDemo::gTestManager->updateCurrentTest(timeStep, timeStep);

    gScene->simulate(timeStep);
    float stepElapsed = stepTimer.stopTimer();
    drawString(5, base, textCol, "Step time: %.5f", stepElapsed);
    base += offset;
    totalTime += stepElapsed;
    drawString(5, base, textCol, "Average: %.5f", totalTime / numSteps);
    ++numSteps;
    base += offset;
  }

  drawString(5, base, textCol, "Number of contacts: %d", gContactFeedback.contactPoints.size());
  base += offset;

  // retrieve profiling data
  if (gScene->getProfiler())
  {
    const bse::Profiler* profiler = gScene->getProfiler();
    bse::ProfilerTask* profileTask = profiler->getRoot();

    int startX = 5;
    base = reportProfilerOutput(profileTask, startX, base, 10, offset, textCol);

    static float elapsedTime = 0;
    elapsedTime += timeStep;
    drawString(5, base, textCol, "Elapsed Time %f", elapsedTime*1000.0f);
    base += offset;
  }

  drawString(5, base, textCol, "Test: %s", BSEDemo::gTestManager->getCurrentTestName());
  base += offset;

  drawString(5, base, textCol, "Space: next test");
  base += offset;

  drawString(5, base, textCol, "A,Z: zoom, ArrKeys: move, B: throw");
  base += offset;

  drawString(5, base, textCol, "D: change render options");
  base += offset;

  drawString(5, base, textCol, "LeftMouse: apply a spring force");
  base += offset;

  if (gInputControl.mEntered)
  {
    drawString(5, base, textCol, "Mouse Entered");
    base += offset;
  }
  if (gInputControl.mLeft)
  {
    drawString(5, base, textCol, "Mouse Left");
    base += offset;
  }

  // Reset some internal state variables.
  gInputControl.update();
}

//---------------------------------------------------------------------------------------------------------------------
void keyboard(int key, int scancode, int action, int mods)
{
  if (action == GLFW_RELEASE)
  {
    gInputControl.keyUp(key);
  }

  if (action == GLFW_PRESS)
  {
    gInputControl.keyDown(key);

    switch (key)
    {
      // Shutdown the test manager and exit the application.
    case GLFW_KEY_ESCAPE:
      BSEDemo::shutDownTestManager();
      destroyDefaultScene();
      bse::toolsShutDown();
      exit(0);
      break;

      // Render options.
    case GLFW_KEY_D:
      updateDrawFlags();
      break;

    case GLFW_KEY_R:
      BSEDemo::gTestManager->restartCurrentTest();
      break;

    case GLFW_KEY_P:
      g_bPaused = !g_bPaused;
      break;
    case GLFW_KEY_B:
      {
        bse::phx::CircleDesc desc;
        desc.radius = 0.05f;

        bse::phx::BodyDesc bombDesc;
        bombDesc.shapesDescs.push_back(&desc);
        bombDesc.mass = 0.1f;
        bombDesc.position = bse::Vec2(-2,1);

        bse::phx::Body* bomb = gScene->createBody(bombDesc);
        bomb->setLinearVelocity(7.5, -0.1f);
      }
      break;

      // Press space to change test.
    case GLFW_KEY_SPACE:
      {
        BSEDemo::gTestManager->startNextTest();
      }
      break;
    case GLFW_KEY_1:
      {
        updatePhysics(1.0f/60.0f);
      }
      break;
    case GLFW_KEY_3:
      {
        BSEDemo::gTestManager->activateNextTool();
      }
      break;
    default:
      break;
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void mouseButton(GLFWwindow* window, int button, int action, int mods, double x, double y)
{  
  gInputControl.mousePos = screenToWorld(x, y);
  gInputControl.mouseButton(button, action == GLFW_PRESS ? 0 : 1);
}

//---------------------------------------------------------------------------------------------------------------------
void mouseMotion(GLFWwindow* window, double x, double y)
{
  gInputControl.mousePos = screenToWorld(x, y);
}

#ifdef WIN32
//---------------------------------------------------------------------------------------------------------------------
// Console handler routine.
BOOL WINAPI consoleHandlerRoutine(DWORD evt)
{
  switch(evt)
  {
  case CTRL_BREAK_EVENT:
      break;
  case CTRL_C_EVENT:
      break;
  case CTRL_CLOSE_EVENT:
      exit(-1);
      break;
  case CTRL_LOGOFF_EVENT:
      break;
  case CTRL_SHUTDOWN_EVENT:
      break;
  }
  return TRUE;
}
#endif // WIN32

//---------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
#ifdef WIN32
  BOOL b = SetConsoleCtrlHandler(consoleHandlerRoutine, TRUE);
#endif

  bse::ToolsDesc toolsDesc;
  toolsDesc.randSeed = getCurrentTime();
  gToolsHandler = bse::toolsInit(&toolsDesc);

////////////////////////////////////////////////////

  createDefaultScene();

  BSEDemo::TestManagerSetup testMgrSetup;
  testMgrSetup.physicsScene = gScene;
  testMgrSetup.debugDraw = &gDebugDraw;
  testMgrSetup.inputControl = &gInputControl;
  testMgrSetup.logger = &gLogger;

  // Initialize the test manager.
  BSEDemo::initializeTestManager(testMgrSetup);

  BSEDemo::gTestManager->startTest(BSEDemo::TEST_ID_BODIES_STACK);

  RenderSceneDesc renderSceneDesc;
  renderSceneDesc.argc = argc;
  renderSceneDesc.argv = argv;
  renderSceneDesc.windowTitle = "bse demo";
  renderSceneDesc.windowHeight = 600;
  renderSceneDesc.windowWidth = 800;
  renderSceneDesc.windowStartX = 0;
  renderSceneDesc.windowStartY = 0;
  renderSceneDesc.updateMode = RENDER_UPDATEMODE_TIMED; // RENDER_UPDATEMODE_CONTINUOS; // RENDER_UPDATEMODE_MANUAL;
  renderSceneDesc.updateTiming = framePeriod;
  renderSceneDesc.camera.fixed = false;
  renderSceneDesc.camera.startZoom = 5;
  renderSceneDesc.keyboardFunc = keyboard;
  renderSceneDesc.frameFunc = redraw;
  renderSceneDesc.mouseFunc = mouseButton;
  renderSceneDesc.mouseMotionFunc = mouseMotion;
//  renderSceneDesc.shutdownFunc = shutDownGame;

  initDrawUtils(renderSceneDesc);

  // Clean up the test manager.
  BSEDemo::shutDownTestManager();

	return 0;
}
