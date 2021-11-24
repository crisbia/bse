#ifndef _H_INCLUDED_TEST_FRAMEWORK
#define _H_INCLUDED_TEST_FRAMEWORK

#include "SettingsStack.h"
#include "TestIDs.h"
#include <vector>

//class bse::phx::Scene;
class bseAIScene;
class PhysicsDebugDraw;

namespace BSEDemo
{

class ContactPointInfo;
class ObjectMouseForce;

typedef enum
{
  MOUSE_LEFT,
  MOUSE_CENTER,
  MOUSE_RIGHT
} MouseButton;

typedef enum
{
  MOUSE_BUTTON_DOWN,
  MOUSE_BUTTON_UP,
} MouseButtonState;


class InputControl
{
public:
  InputControl();

  void update();

  void keyDown(int key);
  void keyUp(int key);
  void specialKeyDown(int key);
  void specialKeyUp(int key);

  void mouseButton(int button, int state);

  bool keys[256];
  int  keyModifiers; // mask for shift (1), alt (2), canc (4)


  int mDx, mDy, mX, mY;
  int mButtons[3];
  bse::Vec2 mousePos;  // position of the mouse in world coordinate.

  bool mEntered, mLeft;
};


class Logger
{
public:
  virtual void log(const char* msg) = 0;
};

typedef unsigned int TestID;

class TestManagerSetup
{
public:
  // bse scenes
  bse::phx::Scene*           physicsScene;
  bseAIScene*         aiScene;

  // input system
  InputControl*       inputControl;

  // debug draw system
  PhysicsDebugDraw*   debugDraw;


  // logging system
  Logger*             logger;
};


/**
 * \brief This class contains all the necessary context information to run a test.
 *
 */
class TestSetup
{
public:
  TestSetup()
  {
  }

public:
};

/**
 * \brief Base class for all the tests. Every test subclass has access to the test manager
 * as a global instance, in order to have visibility on many system objects (the logger, the physics scene, etc...)
 * This allows to not specify any additional parameter to the test when I create it.
 */
class Test
{
  friend class TestManager;
protected:
  Test(TestID id, const char* name) : m_testID(id), m_name(name)
  {
  }
  virtual ~Test() {}
private:
  TestID m_testID;
  const char* m_name;
public:
  TestID getTestID() const { return m_testID; }
  const char* getTestName() const { return m_name; }

//////////////////// test interface
public:
  /**
   * \brief Initialization method, called by the test framework when a test is selected.
   */
  virtual void initialize() = 0;

  /**
   * \brief Runtime method for the test, called once per step. Most tests are just automatic, so they won't
   * probably do anything meaningful here, but some others require to read the user input or to control some
   * of the scene entity, depending on the situation. Most of the AI tests fall in this class.
   */
  virtual void execute(float phxDt, float aiDt) { }

  /**
   * \brief termination method, here the test could get rid of data it creates.
   * The convention here is that the test needs to clean up everything it uses.
   */
  virtual void shutDown() = 0;

  /**
   * \brief debug rendering method, most of the stuff needed to debug a test
   * it's rendered in the main application (primitives, etc) but a test might want to
   * display additional stuff.
   */
  virtual void render(float phxDt, float aiDt) { }

  virtual void handleContact(const ContactPointInfo& contact) { }
protected:
  TestSetup m_testSetup;
};

// \brief Class TestTool
class TestTool
{
public:
  TestTool() :
    m_enabled(true)
  {
  }

  virtual ~TestTool() {}

  void enable();
  void disable();
  bool isEnabled() { return m_enabled; }
  virtual void update(float physicsDt, float aiDt) = 0;
  virtual void render(float physicsDt, float aiDt) = 0;

protected:
  virtual void doEnable() = 0;
  virtual void doDisable() = 0;

  bool m_enabled;
};

// \brief Class TestManager
class TestManager
{
public:
  TestManager(const TestManagerSetup& setup);
  ~TestManager();

  void addTest(Test* test);
  void removeTest(Test* test);

  void addTestTool(TestTool* tool);
  void removeTestTool(TestTool* tool);
public:
  /**
   * \brief Set the subset of tests to execute. If no subset is set, the whole test set is considered.
   * Setting the subset invalidate any internal state: the current test becomes NULL, the subset is overwitten.
   * The next call to startNextTest will start with the first test in the subset.
   * If numTests is 0, the subset is invalidated.
   */
  void setTestSubset(unsigned int numTests, TestID* ids);

  /**
   * \brief Force a test to start, regardless if a subset is used.
   * If a subset is used, it's invalidated. The next call to startNextTest will go to the following one.
   */
  bool startTest(TestID id);

  /**
   * \brief Go to the next test.
   */
  bool startNextTest();

  /**
   * \brief Restart the current test.
   */
  bool restartCurrentTest();

  /**
   * \brief Stop the current test. Shut down the running test.
   */
  void stopCurrentTest();

  Test* getCurrentTest() const { return m_currentTest; }

  void updateCurrentTest(float physicsDt, float aiDt);
  void renderCurrentTest(float physicsDt, float aiDt);

  void handleContact(const ContactPointInfo& contact);

  const char* getCurrentTestName();

  void activateNextTool();
  void resetTool();

public:
  bse::phx::Scene* getPhysicsScene() { return m_setup.physicsScene; }
  PhysicsDebugDraw* getDebugDraw() { return m_setup.debugDraw; }
  InputControl* getInputControl() { return m_setup.inputControl; }
protected:
  bool startTestInternal(Test* test);

  Test* m_currentTest;
  TestTool* m_currentTestTool;

  typedef std::vector<TestTool*> TestTools;
  TestTools m_testTools;

  typedef std::vector<Test*> Tests;
  Tests m_registeredTests;

  Tests m_testsSubset;
  Tests::iterator m_currentTestInSubset;
  SceneSettingsStack m_settingsStack;
  TestManagerSetup m_setup;
  ObjectMouseForce* m_objectPicker;

public:
  void saveSceneSettings();
  void restoreSceneSettings();
};

extern TestManager* gTestManager;

extern void initializeTestManager(const TestManagerSetup& setup);
extern void shutDownTestManager();

} // namespace BSEDemo


#endif // _H_INCLUDED_TEST_FRAMEWORK
