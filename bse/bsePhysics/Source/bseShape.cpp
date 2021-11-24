#include "bseShape.h"
#include "bseBody.h"
#include "bseScene.h"

namespace bse
{
namespace phx
{

#ifdef _DEBUG
  static unsigned int s_globalShapeId = 0;
#endif // _DEBUG

//---------------------------------------------------------------------------------------------------------------------
Shape* Shape::createShape(Scene* scene, ShapeDesc* shapeDesc)
{
  switch (shapeDesc->type)
  {
  case BSE_SHAPE_CIRCLE:
    return createCircle(scene, static_cast<CircleDesc*>(shapeDesc));
  case BSE_SHAPE_BOX:
    return createBox(scene, static_cast<BoxDesc*>(shapeDesc));
  case BSE_SHAPE_POLYGON:
    return createPolygon(scene, static_cast<PolygonDesc*>(shapeDesc));
  default:
    break;
  }

  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
Circle* Shape::createCircle(Scene* scene, CircleDesc* circleShapeDesc)
{
  return new Circle(scene, circleShapeDesc);
}

//---------------------------------------------------------------------------------------------------------------------
Box* Shape::createBox(Scene* scene, BoxDesc* boxShapeDesc)
{
  return new Box(scene, boxShapeDesc);
}

//---------------------------------------------------------------------------------------------------------------------
Polygon* Shape::createPolygon(Scene* scene, PolygonDesc* polygonShapeDesc)
{
  return new Polygon(scene, polygonShapeDesc);
}

//---------------------------------------------------------------------------------------------------------------------
void Shape::destroyShape(Shape* shape)
{
  delete shape;
}

#if 0
//---------------------------------------------------------------------------------------------------------------------
bseCircle* Shape::castToCircleShape()
{
  if (m_desc->type != BSE_SHAPE_SPHERE)
    return 0;

  return static_cast<bseCircle*>(this);
}

//---------------------------------------------------------------------------------------------------------------------
bseBox* Shape::castToBoxShape()
{
  if (m_desc->type != BSE_SHAPE_BOX)
    return 0;

  return static_cast<bseBox*>(this);
}

//---------------------------------------------------------------------------------------------------------------------
bsePolygon* Shape::castToPolygonShape()
{
  if (m_desc->type != BSE_SHAPE_POLYGON)
    return 0;
  return static_cast<bsePolygon*>(this);
}
#endif

//---------------------------------------------------------------------------------------------------------------------
Shape::Shape(Scene* scene, ShapeDesc* shapeDesc) :

  m_userData(0),  m_desc(shapeDesc), m_scene(scene), m_body(0), m_position(0.0f,0.0f), m_orientation(0), m_rotationMatrix(0), m_material(0)
{

  setMoved();
  setChanged();

#ifdef _DEBUG
  m_shapeId = s_globalShapeId++;
#endif // _DEBUG
}

//---------------------------------------------------------------------------------------------------------------------
Shape::~Shape()
{
}

//---------------------------------------------------------------------------------------------------------------------
void Shape::updateAABB()
{
  doUpdateAABB();
  m_hasChanged = false;
  m_hasMoved = false;
  if (m_body)
  {
    // Signal the body that the shape's aabb has been refreshed.
    m_body->setShapeClean(this);
  }
}

//---------------------------------------------------------------------------------------------------------------------
Material* Shape::getMaterial() const
{
  if (m_material)
  {
    return m_material;
  }

  // assumption: default material doesn't need to be set explicitly
  return m_scene->getDefaultMaterial();
}

//---------------------------------------------------------------------------------------------------------------------
void Shape::setBody(Body* body)
{
  m_body = body;
}

//---------------------------------------------------------------------------------------------------------------------
bse::Vec2 Shape::getPosition() const
{
  if (m_body)
  {
    return m_body->getPosition() + bseMul(m_body->getRotationMatrix(), m_position);
  }

  // if it's static, consider "localPos" as a global value
  return m_position;
}

//---------------------------------------------------------------------------------------------------------------------
bse::Real Shape::getOrientation() const
{
  if (m_body)
  {
    return m_body->getOrientation();
  }

  return m_orientation;
}

//---------------------------------------------------------------------------------------------------------------------
Mat22 Shape::getRotationMatrix() const
{
  if (m_body)
  {
    return m_body->getRotationMatrix();
  }

  return m_rotationMatrix;
}

//---------------------------------------------------------------------------------------------------------------------
bool Shape::isStatic() const
{
  return m_body->isStatic();
}

//---------------------------------------------------------------------------------------------------------------------
//////////////////////////////////////
// circle
//////////////////////////////////////
void Circle::doUpdateAABB()
{
  // circle... easy.. I don't care about orientation
  bse::Vec2 globalPos = getPosition();
  m_aabb.low = bse::Vec2(globalPos.x - getRadius(), globalPos.y - getRadius());
  m_aabb.high = bse::Vec2(globalPos.x + getRadius(), globalPos.y + getRadius());
}

//---------------------------------------------------------------------------------------------------------------------
bool Circle::contains(const bse::Vec2& point) const
{
  bse::Vec2 p = point - getPosition();
  return p.sqrmag() < m_radius*m_radius;
}

//---------------------------------------------------------------------------------------------------------------------
//////////////////////////////////////
// box
//////////////////////////////////////
void Box::doUpdateAABB()
{
  // build a "point" bb in the global position
  Mat22 ori = getRotationMatrix();

//  globalPos = pos + bodyOri * localPos

  bse::Vec2 pos = getPosition();

  AABB aabb(pos, pos);

  // enclose every of the four points in the bb
  bse::Real widthHalf = m_dims.x/2.0f;
  bse::Real heightHalf = m_dims.y/2.0f;

  aabb.enclose(pos + bseMulT(ori, bse::Vec2(widthHalf, heightHalf)));
  aabb.enclose(pos + bseMulT(ori, bse::Vec2(-widthHalf, heightHalf)));
  aabb.enclose(pos + bseMulT(ori, bse::Vec2(-widthHalf, -heightHalf)));
  aabb.enclose(pos + bseMulT(ori, bse::Vec2(widthHalf, -heightHalf)));

  m_aabb.low  = aabb.low;
  m_aabb.high = aabb.high;
}

//---------------------------------------------------------------------------------------------------------------------
bool Box::contains(const bse::Vec2& point) const
{
  // bring the point in box space.
  bse::Vec2 pBox = bseMulT(getRotationMatrix(), (point-getPosition()));
  bse::Vec2 minusHalfDims(-m_halfDims.x, -m_halfDims.y);

  AABB tempAABB(minusHalfDims, m_halfDims);
  return tempAABB.contains(pBox);
}

//---------------------------------------------------------------------------------------------------------------------
void Box::getEdgePoints(int id, bse::Vec2& start, bse::Vec2& end) const
{
  // TODO I should really cache some of this stuff.
  BSE_ASSERT(id>=0 && id<4);

  bse::Vec2 pos = getPosition();
  Mat22 ori = getRotationMatrix();
  bse::Vec2 half = getHalfDims();

  bse::Vec2 edge;
  switch (id)
  {
  case 0:
    end   = pos + bseMul(ori, bse::Vec2(half.x, half.y));
    start = pos + bseMul(ori, bse::Vec2(half.x, -half.y));
    break;
  case 1:
    end   = pos + bseMul(ori, bse::Vec2(-half.x, half.y));
    start = pos + bseMul(ori, bse::Vec2(half.x, half.y));
    break;
  case 2:
    end   = pos + bseMul(ori, bse::Vec2(-half.x, -half.y));
    start = pos + bseMul(ori, bse::Vec2(-half.x, half.y));
    break;
  case 3:
    end   = pos + bseMul(ori, bse::Vec2(half.x, -half.y));
    start = pos + bseMul(ori, bse::Vec2(-half.x, -half.y));
    break;
  }
}

bse::Vec2 Box::getEdge(int id) const
{
  // TODO I should really cache some of this stuff.
  BSE_ASSERT(id>=0 && id<4);

  bse::Vec2 pos = getPosition();
  Mat22 ori = getRotationMatrix();
  bse::Vec2 half = getHalfDims();

  bse::Vec2 edge;
  switch (id)
  {
  case 0:
    edge = bseMul(ori, bse::Vec2(half.x, half.y)) - bseMul(ori, bse::Vec2(half.x, -half.y));
    break;
  case 1:
    edge = bseMul(ori, bse::Vec2(-half.x, half.y)) - bseMul(ori, bse::Vec2(half.x, half.y));
    break;
  case 2:
    edge = bseMul(ori, bse::Vec2(-half.x, -half.y)) - bseMul(ori, bse::Vec2(-half.x, half.y));
    break;
  case 3:
    edge = bseMul(ori, bse::Vec2(half.x, -half.y)) - bseMul(ori, bse::Vec2(-half.x, -half.y));
    break;
  }

  return edge;
}

//////////////////////////////////////
// polygon
//////////////////////////////////////

//---------------------------------------------------------------------------------------------------------------------
void Polygon::doUpdateAABB()
{
  // vertexes are stored in local space... need to transform all of them...

  // build a "point" bb in the global position
  bse::Vec2 pos = getPosition();
  Mat22 ori = getRotationMatrix();
  AABB aabb(pos, pos);

  // enclose every of the points in the bb
  for (bseVertexesListIter vertIter=m_vertexes.begin(); vertIter!=m_vertexes.end(); vertIter++)
    aabb.enclose(pos + bseMul(ori, (*vertIter)));

  m_aabb.low = aabb.low;
  m_aabb.high = aabb.high;
}

//---------------------------------------------------------------------------------------------------------------------
bool Polygon::contains(const bse::Vec2& point) const
{
  // bring the point in polygon space
  bse::Vec2 pPoly = getPosition();
  Mat22 oPoly = getRotationMatrix();
  bse::Vec2 pos = bseMulT(oPoly, (point-pPoly));

  // I need the normal for each edge, as in 3D, defined by the vertexes order.
  // The polygon needs to be convex. TODO put a test of convexity in place, and assert if it fails.
  bse::Vec2 normal, edge, pRel;
  bse::Vec2 minNormal;

  const bseVertexesList vList = getVertexesList();
  bse::UInt numVerts = static_cast<bse::UInt>(vList.size());
  for (bse::UInt vIndex=0; vIndex<numVerts; ++vIndex)
  {
    // edges and normals could be pre-computed, at least in local space
    edge = vList[(vIndex+1)%numVerts] - vList[vIndex];
    normal = bseCross(edge, 1.0f);
    normal.normalize();

    // check the point
    pRel = pos - vList[vIndex];
    bse::Real dot = bseDot(pRel, normal);
    if (dot>-0.0f)
      return false; // point outside the polygon
  }

  return true;
}

}
}
