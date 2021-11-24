#include "TestUtils.h"
#include "DrawUtils.h"

namespace BSEDemo
{
//---------------------------------------------------------------------------------------------------------------------
void createGround(bse::phx::Scene* scene, bse::Real angle, const bse::Vec2& groundDims)
{
  bse::phx::BoxDesc groundShapeDesc;
  groundShapeDesc.dims = groundDims;

  bse::phx::BodyDesc groundDesc;
  groundDesc.flags |= BSE_BODYFLAG_STATIC;
  groundDesc.orientation = angle;

  groundDesc.shapesDescs.push_back(&groundShapeDesc);
  bse::phx::Body* ground = scene->createBody(&groundDesc);

  ground->setPosition(0,-groundDims.y * 0.5f);
}

//---------------------------------------------------------------------------------------------------------------------
// x, y is the lower left corner
void createRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY)
{
  bse::phx::Scene* gScene = scene;

  bse::phx::BoxDesc x_boxDesc;
  x_boxDesc.dims.x = sizeX;
  x_boxDesc.dims.y = 0.1f; // just a bit of thickness

  bse::phx::BoxDesc y_boxDesc;
  y_boxDesc.dims.x = 0.1f;
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
// x, y is the lower left corner
void createPolygonalRoom(bse::phx::Scene* scene, const bse::Real x, const bse::Real y, const bse::Real sizeX, const bse::Real sizeY)
{
  bse::phx::PolygonDesc polyDesc[4];

  bse::Vec2 sizes[4] = {
    bse::Vec2(0.1f,  sizeY),
    bse::Vec2(0.1f,  sizeY),
    bse::Vec2(sizeX, 0.1f),
    bse::Vec2(sizeX, 0.1f)
  };

  bse::Vec2 pos[4] = {
    bse::Vec2(x-0.05f,           y + sizeY/2.0f),
    bse::Vec2(x+0.05f + sizeX,   y + sizeY/2.0f),
    bse::Vec2(x + sizeX / 2.0f,  y+0.05f + sizeY),
    bse::Vec2(x + sizeX / 2.0f,  y-0.05f)
  };

  for (int i=0; i<4; ++i)
  {
    bse::Vec2 dims = sizes[i]*0.5f;
    polyDesc[i].vertList.push_back(bse::Vec2(dims.x, dims.y));
    polyDesc[i].vertList.push_back(bse::Vec2(-dims.x, dims.y));
    polyDesc[i].vertList.push_back(bse::Vec2(-dims.x, -dims.y));
    polyDesc[i].vertList.push_back(bse::Vec2(dims.x, -dims.y));

    bse::phx::BodyDesc bodyDesc;
    bodyDesc.flags |= BSE_BODYFLAG_STATIC;
    bodyDesc.shapesDescs.push_back(&polyDesc[i]);
    bse::phx::Body* body = scene->createBody(&bodyDesc);
    body->setPosition(pos[i]);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void createPolygon(bse::phx::PolygonDesc& polyDesc, const bse::Real extRadius, const int numEdges)
{
  for (int i=0; i<numEdges; ++i)
  {
    bse::Real angle = i*2.0f*bse_pi/(bse::Real)numEdges;
    bse::Vec2 vert(extRadius*cos(angle), extRadius*sin(angle));
    polyDesc.vertList.push_back(vert);
  }
}

//---------------------------------------------------------------------------------------------------------------------
ObjectMouseForce::ObjectMouseForce() :
  m_currentObject(0),
  m_strength(10)
{
  m_objectState.save(0);
  m_scene = gTestManager->getPhysicsScene();
}

//---------------------------------------------------------------------------------------------------------------------
void ObjectMouseForce::update(float physicsDt, float aiDt)
{
  InputControl* inputControl = gTestManager->getInputControl();
  bse::Vec2 position = inputControl->mousePos;

  bse::Vec2 oldPos = m_position;
  m_position = position;

  if (inputControl->mButtons[MOUSE_LEFT]==MOUSE_BUTTON_UP)
  {
    m_currentObject = 0;
  }

  if (!m_currentObject)
  {
    // Try to pick an object, but only if the mouse button is down (for efficiency).
    if (inputControl->mButtons[MOUSE_LEFT]==MOUSE_BUTTON_DOWN)
    {
      m_currentObject = pick();

      // Update the state of the object.
      m_objectState.save(m_currentObject);
      if (m_currentObject)
      {
        m_offset = m_currentObject->getBody()->getLocalSpacePoint(m_position);
      }
    }
  }
  else
  {
    // If an object is already picked, check the mouse button,
    // and apply a spring.
    if (inputControl->mButtons[MOUSE_LEFT]==MOUSE_BUTTON_DOWN)
    {
      // Basic picker. Just apply a spring.
      bse::Vec2 point = m_currentObject->getBody()->getWorldSpacePoint(m_offset);
      bse::Vec2 force = (m_position - point) * m_strength;

      m_currentObject->getBody()->addForceAtRelPoint(force, m_offset);
    }

    // Update the object state.
    m_objectState.save(m_currentObject);
  }

}

//---------------------------------------------------------------------------------------------------------------------
void ObjectMouseForce::render(float physicsDt, float aiDt)
{
  PhysicsDebugDraw* debugDraw = gTestManager->getDebugDraw();
  debugDraw->drawCircle(m_position, 0.02f, 12, Color(1,1,1));
  if (m_currentObject)
  {
    // Circle in the anchor point of the picked object.
    bse::Vec2 anchorPoint = m_currentObject->getBody()->getWorldSpacePoint(m_offset);
    debugDraw->drawCircle(anchorPoint, 0.02f, 12, Color(1,1,1));
    // Line connecting the body and the picker.
    debugDraw->drawLine(anchorPoint, m_position, Color(1,1,1));
  }
}

//---------------------------------------------------------------------------------------------------------------------
bse::phx::Shape* ObjectMouseForce::pick()
{
  reset();
  bse::phx::Shape* shape = m_scene->pointCollision(m_position);
  return shape;
}

//---------------------------------------------------------------------------------------------------------------------
ObjectSelector::ObjectSelector() :
  m_currentObject(0)
{
  m_scene = gTestManager->getPhysicsScene();
}

//---------------------------------------------------------------------------------------------------------------------
void ObjectSelector::update(float physicsDt, float aiDt)
{
  InputControl* inputControl = gTestManager->getInputControl();
  m_position = inputControl->mousePos;
  if (inputControl->keys[127] && m_currentObject)
  {
    m_scene->releaseBody(m_currentObject);
    m_currentObject = 0;
  }

  // Try to pick an object, but only if the mouse button is down (for efficiency).
  if (inputControl->mButtons[MOUSE_LEFT]==MOUSE_BUTTON_DOWN)
  {
    m_currentObject = pick();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void ObjectSelector::render(float physicsDt, float aiDt)
{
  Color selCol(1,1,1);
  PhysicsDebugDraw* debugDraw = gTestManager->getDebugDraw();
  debugDraw->drawCross(m_position, 0.1f, selCol);
  if (m_currentObject)
  {
    // Draw a contour around the object's shapes.
    bse::phx::ShapesList& shapes = m_currentObject->getShapes();
    for (bse::phx::ShapesList::iterator iter = shapes.begin(); iter != shapes.end(); ++iter)
    {
      bse::phx::Shape* shape = (*iter);
      debugDraw->drawShape(shape, selCol, SHAPE_DRAW_WIREFRAME, 1.025f);
      debugDraw->drawShape(shape, selCol, SHAPE_DRAW_WIREFRAME, 1.05f);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
bse::phx::Body* ObjectSelector::pick()
{
  reset();
  bse::phx::Shape* shape = m_scene->pointCollision(m_position);
  if (shape)
  {
    return shape->getBody();
  }

  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
void ObjectState::save(bse::phx::Shape* shape)
{
  this->shape = shape;
  if (shape)
  {
    bse::phx::Body* body = shape->getBody();
    // assert(body);
    position = body->getPosition();
    orientation = body->getOrientation();
    linearVelocity = body->getLinearVelocity();
    angularVelocity = body->getAngularVelocity();
  }
}

} // namespace BSEDemo
