#include "TestFramework.h"

#include "TestUtils.h"
#include "DrawUtils.h"
#include "TestIDs.h"
#include "PhysicsTests.h"

#include "bseScene.h"

namespace BSEDemo
{

//---------------------------------------------------------------------------------------------------------------------
InputControl::InputControl() :
  keyModifiers(0),
  mDx(0), mDy(0),
  mX(-1), mY(-1)
{
  for (int i=0; i<256; ++i)
  {
    keys[i] = false;
  }

  mButtons[0] = mButtons[1] = mButtons[2] = MOUSE_BUTTON_UP;
  mEntered = mLeft = false;
}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::update()
{
  mEntered = false;
  mLeft = false;
}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::keyDown(int key)
{
  BSE_ASSERT(key>=0 && key<256);
  keys[key] = true;
}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::keyUp(int key)
{
  BSE_ASSERT(key>=0 && key<256);
  keys[key] = false;
}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::specialKeyDown(int key)
{

}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::specialKeyUp(int key)
{

}

//---------------------------------------------------------------------------------------------------------------------
void InputControl::mouseButton(int button, int state)
{
  BSE_ASSERT(button>=0 && button<3);
  BSE_ASSERT(state == MOUSE_BUTTON_UP || state == MOUSE_BUTTON_DOWN);
  mButtons[button] = state;
}

TestManager* gTestManager = 0;

//---------------------------------------------------------------------------------------------------------------------
void initializeTestManager(const TestManagerSetup& setup)
{
  if (gTestManager)
    delete gTestManager;

  gTestManager = new TestManager(setup);

  // Add all the tests here.
  gTestManager->addTest(new MinimalTest());
  gTestManager->addTest(new StackTest());
  gTestManager->addTest(new SAPTest());
  gTestManager->addTest(new RayCastTest());
  gTestManager->addTest(new PyramidTest());
  gTestManager->addTest(new PolygonsTest());
  gTestManager->addTest(new StressTest());
  gTestManager->addTest(new FrictionTest());

  gTestManager->setTestSubset(0, 0);

  // Add test tool.
  gTestManager->addTestTool(new ObjectMouseForce());
  gTestManager->addTestTool(new ObjectSelector());

  gTestManager->activateNextTool();
}

//---------------------------------------------------------------------------------------------------------------------
void shutDownTestManager()
{
  delete gTestManager;
  gTestManager = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void TestTool::enable()
{
  m_enabled = true;
  doEnable();
}

//---------------------------------------------------------------------------------------------------------------------
void TestTool::disable()
{
  m_enabled = false;
  doDisable();
}

//---------------------------------------------------------------------------------------------------------------------
///
/// Class: TestManager
///
TestManager::TestManager(const TestManagerSetup& setup) :
  m_currentTest(0),
  m_currentTestTool(0),
  m_settingsStack(setup.physicsScene),
  m_setup(setup)
{
  m_registeredTests.resize(NUM_TEST_IDS);
}

//---------------------------------------------------------------------------------------------------------------------
TestManager::~TestManager()
{
  // The current test could have some local memory allocated. If not shut down properly, it would leak.
  if (m_currentTest)
  {
    m_currentTest->shutDown();
    m_currentTest = 0;
  }

  for (Tests::iterator iter = m_registeredTests.begin(); iter != m_registeredTests.end(); ++iter)
  {
    delete (*iter);
  }

  m_registeredTests.clear();
  m_testsSubset.clear();

  for (TestTools::iterator iter = m_testTools.begin(); iter != m_testTools.end(); ++iter)
  {
    delete (*iter);
  }

  m_testTools.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::setTestSubset(unsigned int numTests, TestID* ids)
{
  if (numTests==0)
  {
    m_testsSubset = m_registeredTests;
    m_currentTestInSubset = m_testsSubset.end();
  }
  else
  {
    if (m_currentTest)
    {
      m_currentTest->shutDown();
      m_currentTest = 0;
    }

    m_testsSubset.clear();

    for (unsigned int iTest=0; iTest<numTests; ++iTest)
    {
      BSE_ASSERT(ids[iTest]<m_registeredTests.size());
      m_testsSubset.push_back(m_registeredTests[ids[iTest]]);
    }

    m_currentTestInSubset = m_testsSubset.end();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::activateNextTool()
{
  TestTool* currentTool = m_currentTestTool;
  resetTool();

  size_t nextTool = 0;
  if (currentTool)
  {
    for (size_t i = 0; i<m_testTools.size(); ++i)
    {
      if (m_testTools[i] == currentTool)
      {
        nextTool = ++i;
        if (nextTool == m_testTools.size())
        {
          nextTool = 0;
        }
      }
    }
  }

  m_currentTestTool = m_testTools[nextTool];
  m_currentTestTool->enable();
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::resetTool()
{
  if (m_currentTestTool)
  {
    m_currentTestTool->disable();
    m_currentTestTool = 0;
  }
}

//---------------------------------------------------------------------------------------------------------------------
const char* TestManager::getCurrentTestName()
{
  if (m_currentTest)
    return m_currentTest->getTestName();
  return "Null";
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::updateCurrentTest(float physicsDt, float aiDt)
{
  if (m_currentTest)
  {
    m_currentTest->execute(physicsDt, aiDt);
  }

#if 0
  for (size_t iTool = 0; iTool<m_testTools.size(); ++iTool)
  {
    if (m_testTools[iTool]->isEnabled())
    {
      m_testTools[iTool]->update(physicsDt, aiDt);
    }
  }
#endif

  if (m_currentTestTool)
  {
    m_currentTestTool->update(physicsDt, aiDt);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::renderCurrentTest(float physicsDt, float aiDt)
{
  if (m_currentTest)
    m_currentTest->render(physicsDt, aiDt);

  if (m_currentTestTool)
  {
    m_currentTestTool->render(physicsDt, aiDt);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::handleContact(const ContactPointInfo& contact)
{
  if (m_currentTest)
    m_currentTest->handleContact(contact);
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::addTest(Test* test)
{
  BSE_ASSERT(m_registeredTests.size() >= test->getTestID());
  m_registeredTests[test->getTestID()] = test;
}

//---------------------------------------------------------------------------------------------------------------------
bool TestManager::startTest(TestID id)
{
  BSE_ASSERT(id>=0 && id<NUM_TEST_IDS);
  Test* test = m_registeredTests[id];
  if (startTestInternal(test))
  {
      // Update the current test index (in the subset).
    Test* test = m_currentTest;
    Tests::iterator iter=m_testsSubset.begin();
    for ( ; iter!=m_testsSubset.end(); ++iter)
      if ((*iter)==test)
        break;

    m_currentTestInSubset = iter; // Takes the end if the search fails... but it shouldn't!

    return true;
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool TestManager::startTestInternal(Test* test)
{
  // Shutdown the current test.
  if (m_currentTest)
  {
    m_currentTest->shutDown();
    m_currentTest = 0;
  }

  if (!test)
    return false;

  TestID id = test->getTestID();

  if (id >=0 && id < NUM_TEST_IDS)
  {
    Test* test = m_registeredTests.at(id);
    m_currentTest = test;
    if (test)
    {
      test->initialize();
      getPhysicsScene()->simulate(0);
      return true;
    }
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::removeTest(Test* test)
{
  // nothing atm
  BSE_ASSERT(test && test->getTestID()<NUM_TEST_IDS);
  BSE_ASSERT(m_registeredTests[test->getTestID()]!=0);

  m_registeredTests[test->getTestID()] = 0;

  // Make sure to remove the test from the subset
  for (Tests::iterator iter; iter!=m_testsSubset.end(); ++iter)
    if ((*iter)==test)
    {
      (*iter) = 0;
      break;
    }

  delete test;
}

//---------------------------------------------------------------------------------------------------------------------
bool TestManager::startNextTest()
{
  if (m_testsSubset.empty())
  {
    return false;
  }

  // This is a special case. When the test set changes, I set the end
  // to indicate an undefined state.
  if (m_currentTestInSubset != m_testsSubset.end())
  {
    do
    {
      m_currentTestInSubset++;

      // skip null test.
    }
    while (m_currentTestInSubset != m_testsSubset.end() && (*m_currentTestInSubset)==0);
  }

  if (m_currentTestInSubset == m_testsSubset.end())
  {
    m_currentTestInSubset = m_testsSubset.begin();
  }

  Test* nextTest = (*m_currentTestInSubset);
  startTestInternal(nextTest);

  return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool TestManager::restartCurrentTest()
{
  // Just start the same one (if valid).
  startTestInternal(m_currentTest);
  return true;
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::saveSceneSettings()
{
  m_settingsStack.push();
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::restoreSceneSettings()
{
  m_settingsStack.pop();
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::addTestTool(TestTool* tool)
{
  m_testTools.push_back(tool);
}

//---------------------------------------------------------------------------------------------------------------------
void TestManager::removeTestTool(TestTool* tool)
{
  bool found = false;
  for (size_t iTool=0; iTool<m_testTools.size(); ++iTool)
  {
    if (m_testTools[iTool]==tool)
    {
      if (m_currentTestTool == tool)
      {
        resetTool();
      }

      m_testTools[iTool] = m_testTools.back();

      found = true;
      break;
    }
  }

  if (found)
  {
    m_testTools.pop_back();
  }
}

} // namespace BSEDemo
