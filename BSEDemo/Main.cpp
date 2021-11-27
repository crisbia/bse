///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// BSEDemo
// Cristian Bianchi
// 2008-2011
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <vector>

// glut includes
#include <glut.h>

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

bse::Int width = 800;
bse::Int height = 600;
bse::Int framePeriod = 16;
bse::Int mainWindow;
bse::Real viewZoom = 2;
bse::Real viewX = 0.0f;
bse::Real viewY = -1.0f;
int tx, ty, tw, th;

//---------------------------------------------------------------------------------------------------------------------
void reshape(bse::Int w, bse::Int h)
{
  width = w;
  height = h;

  bse::Int curr_x = 0;
  bse::Int curr_y = 0;
  bse::Int curr_w = glutGet( GLUT_WINDOW_WIDTH );
  bse::Int curr_h = glutGet( GLUT_WINDOW_HEIGHT );

  tx = curr_x;
  ty = curr_y;
  tw = curr_w;
  th = curr_h;

  glViewport( tx, ty, tw, th );

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double ratio = (double)tw / (double)th;

  gluOrtho2D(viewZoom * (viewX - ratio), viewZoom * (ratio + viewX),
    viewZoom * (viewY - 0.1), viewZoom * (viewY + 1.9));
}

//---------------------------------------------------------------------------------------------------------------------
// Convert a window coordinate point into graphics world coordinates.
bse::Vec2 screenToWorld(bse::Int x, bse::Int y)
{
  bse::Vec2 p;
  bse::Real ratio = bse::Real(tw) / bse::Real(th);
  bse::Real u = x / bse::Real(tw);
  bse::Real v = (th - y) / bse::Real(th);
  p.x = viewZoom * (viewX - ratio) * (1.0f - u) + viewZoom * (ratio + viewX) * u;
  p.y = viewZoom * (viewY - 0.1f) * (1.0f - v) + viewZoom * (viewY + 1.9f) * v;
  return p;
}

//---------------------------------------------------------------------------------------------------------------------
// This is used to control the frame rate (60Hz).
void timerCallback(int)
{
  glutSetWindow(mainWindow);
  glutPostRedisplay();
  glutTimerFunc(framePeriod, timerCallback, 0);
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

  glutSwapBuffers();

  // Reset some internal state variables.
  gInputControl.update();
}

//---------------------------------------------------------------------------------------------------------------------
void keyUp(unsigned char key, int x, int y)
{
  gInputControl.keyUp(key);
}

//---------------------------------------------------------------------------------------------------------------------
void keyDown(unsigned char key, int x, int y)
{
  gInputControl.keyDown(key);

  switch (key)
  {
    // Shutdown the test manager and exit the application.
  case 27:
    BSEDemo::shutDownTestManager();
    destroyDefaultScene();
    bse::toolsShutDown();
    exit(0);
    break;

    // Render options.
  case 'd':
    updateDrawFlags();
    break;

    // Zoom in.
  case 'a':
    viewZoom = bseMax(viewZoom - 0.1f, 0.1f);
    reshape(width, height);
    break;

    // Zoom out.
  case 'z':
    viewZoom = bseMin(viewZoom + 0.1f, 100.0f);
    reshape(width, height);
    break;

  case 'r':
    BSEDemo::gTestManager->restartCurrentTest();
    break;

  case 'p':
    g_bPaused = !g_bPaused;
    break;
  case 'b':
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
  case ' ':
    {
      BSEDemo::gTestManager->startNextTest();
    }
    break;
  case '1':
    {
      updatePhysics(1.0f/60.0f);
    }
    break;
  case '3':
    {
      BSEDemo::gTestManager->activateNextTool();
    }
    break;
  default:
    break;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void keyDownSpecial(int key, int x, int y)
{
  switch (key)
  {
  case GLUT_KEY_LEFT:
    viewX += 0.1f;
    reshape(width, height);
    break;

  case GLUT_KEY_RIGHT:
    viewX -= 0.1f;
    reshape(width, height);
    break;

  case GLUT_KEY_DOWN:
    viewY += 0.1f;
    reshape(width, height);
    break;

  case GLUT_KEY_UP:
    viewY -= 0.1f;
    reshape(width, height);
    break;

  case GLUT_KEY_HOME:
    viewZoom = 20.0f;
    viewX = 0.0f;
    viewY = 0.0f;
    reshape(width, height);
    break;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void keyUpSpecial(int key, int x, int y)
{
  switch (key)
  {
  case GLUT_KEY_LEFT:
    break;
  case GLUT_KEY_RIGHT:
    break;
  case GLUT_KEY_DOWN:
    break;
  case GLUT_KEY_UP:
    break;
  case GLUT_KEY_HOME:
    break;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void mouseButton(bse::Int button, bse::Int state, bse::Int x, bse::Int y)
{
  gInputControl.mousePos = screenToWorld(x, y);
  gInputControl.mX = x;
  gInputControl.mY = y;
  gInputControl.mouseButton(button, state);
}

//---------------------------------------------------------------------------------------------------------------------
void passiveMouseMotion(bse::Int x, bse::Int y)
{
  gInputControl.mousePos = screenToWorld(x, y);
  gInputControl.mX = x;
  gInputControl.mY = y;
}

//---------------------------------------------------------------------------------------------------------------------
void activeMouseMotion(bse::Int x, bse::Int y)
{
  gInputControl.mousePos = screenToWorld(x, y);
  gInputControl.mX = x;
  gInputControl.mY = y;
}

#if 0 // This callback doesn't work as expected on Win32.
//---------------------------------------------------------------------------------------------------------------------
void mouseEntry(int state)
{
  switch (state)
  {
  case GLUT_ENTERED:
    printf("Mouse entered\n");
    gInputControl.mEntered = true;
    break;
  case GLUT_LEFT:
    printf("Mouse left\n");
    gInputControl.mLeft = true;
    break;
  }
}
#endif

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

//////////// setup glut
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(width, height);
  mainWindow = glutCreateWindow("bse demo");

  // This function is freeglut specific, not glut standard.
#if 0
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

  glutDisplayFunc(redraw);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyDown);
  glutKeyboardUpFunc(keyUp);
  glutSpecialFunc(keyDownSpecial);
  glutSpecialUpFunc(keyUpSpecial);
  glutMouseFunc(mouseButton);
  glutMotionFunc(activeMouseMotion);         // active motion, fired when a button is pressed and there's motion.
  glutPassiveMotionFunc(passiveMouseMotion); // passive motion.

  // Use a timer to control the frame rate.
  glutTimerFunc(framePeriod, timerCallback, 0);


  BSEDemo::TestManagerSetup testMgrSetup;
  testMgrSetup.physicsScene = gScene;
  testMgrSetup.debugDraw = &gDebugDraw;
  testMgrSetup.inputControl = &gInputControl;
  testMgrSetup.logger = &gLogger;

  // Initialize the test manager.
  BSEDemo::initializeTestManager(testMgrSetup);

  BSEDemo::gTestManager->startTest(BSEDemo::TEST_ID_BODIES_STACK);

//  BSEDemo::gTestManager->startTest(BSEDemo::TEST_ID_STRESS_TEST);
//  BSEDemo::gTestManager->startTest(BSEDemo::TEST_ID_FRICTION);

  glutMainLoop();

  // Clean up the test manager.
  BSEDemo::shutDownTestManager();

	return 0;
}
