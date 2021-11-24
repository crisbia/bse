#ifndef _BSE_SHAPE_INCLUDED
#define _BSE_SHAPE_INCLUDED

#include "bseMath.h"
#include "bseAABB.h"
#include <list>
#include <vector>

namespace bse
{
namespace phx
{

class Scene;
class Body;
class Material;
class Circle;
class Box;
class Polygon;

// Shape is a factory for the other shapes
typedef unsigned int bseShapeFlags;
typedef std::vector<bse::Vec2> bseVertexesList;
typedef std::vector<bse::Vec2>::iterator bseVertexesListIter;

//---------------------------------------------------------------------------------------------------------------------
typedef enum bseShapeType_enum
{
  BSE_SHAPE_CIRCLE,
  BSE_SHAPE_BOX,
  BSE_SHAPE_POLYGON,
  NUM_BSE_SHAPE_TYPES
} bseShapeType;

//---------------------------------------------------------------------------------------------------------------------
class ShapeDesc
{
friend class Shape;
protected:
  ShapeDesc(bseShapeType shapeType) : type(shapeType), flags(0)
  {
  }
public:
  bseShapeType type;
  bseShapeFlags flags;
};

//---------------------------------------------------------------------------------------------------------------------
class CircleDesc : public ShapeDesc
{
public:
  CircleDesc() : ShapeDesc(BSE_SHAPE_CIRCLE) { }
  bse::Real radius;
};

//---------------------------------------------------------------------------------------------------------------------
class BoxDesc : public ShapeDesc
{
public:
  BoxDesc() : ShapeDesc(BSE_SHAPE_BOX) , dims(0.0f, 0.0f) { }
  bse::Vec2 dims; // box dimensions
};

//---------------------------------------------------------------------------------------------------------------------
class PolygonDesc : public ShapeDesc
{
public:
  PolygonDesc() : ShapeDesc(BSE_SHAPE_POLYGON) { }
  bseVertexesList vertList;
};


//---------------------------------------------------------------------------------------------------------------------
// Shape
//---------------------------------------------------------------------------------------------------------------------
class Shape
{
public:
  static Shape* createShape(Scene* scene, ShapeDesc* shapeDesc);
  static Circle* createCircle(Scene* scene, CircleDesc* circleShapeDesc);
  static Box* createBox(Scene* scene, BoxDesc* ShapeDesc);
  static Polygon* createPolygon(Scene* scene, PolygonDesc* polygonShapeDesc);
  static void destroyShape(Shape* shape);

protected:
  Shape(Scene* scene, ShapeDesc* shapeDesc);
  virtual ~Shape();

public:
  Body* getBody() const { return m_body; }
  void setBody(Body* body);

  void setPosition(const bse::Vec2& position)
  {
    m_position = position;
    setMoved();
  }

  void setPosition(bse::Real x, bse::Real y)
  {
    setPosition(bse::Vec2(x,y));
  }

  void setOrientation(bse::Real angle)
  {
    m_orientation = angle;
    m_rotationMatrix.set(angle);
    setMoved();
  }

  bse::Vec2 getPosition() const;
  bse::Real getOrientation() const;
  Mat22 getRotationMatrix() const;

  bool hasMoved() const { return m_hasMoved; }
  bool hasChanged() const { return m_hasChanged; }

  void setMoved() { m_hasMoved = true; }
  void setChanged() { m_hasChanged = true; }
  void resetMoved() { m_hasMoved = false; }
  void resetChanged() { m_hasChanged = false; }

  void updateAABB();
  virtual bool contains(const bse::Vec2& point) const = 0;

  const AABB* getAABB() const { return &m_aabb; }
  void getAABB(AABB& aabb) const { aabb = m_aabb; }

  bseShapeType getType() const { return m_desc->type; }

  bool isStatic() const;

  /// user data management
public:
  bse::UserData getUserData() { return m_userData; }
  void setUserData(bse::UserData userData) { m_userData = userData; }
protected:
  bse::UserData m_userData;

protected:
  virtual void doUpdateAABB() = 0;

  ShapeDesc* m_desc; // for the main shape, I store a pointer to the desc, because should be of a subtype
  Scene* m_scene;

  Body* m_body;
  bse::Vec2 m_position;
  bse::Real m_orientation;
  Mat22 m_rotationMatrix;

protected:
  AABB m_aabb;
  bool m_hasChanged;
  bool m_hasMoved;

///////////////////// Material Management
public:
  Material* getMaterial() const;
  void setMaterial(Material* material) { m_material = material; }
protected:
  Material* m_material;  // this is optional, if not set, the scene default is used.

#ifdef _DEBUG
  unsigned int m_shapeId;
public:
  unsigned int getShapeId() const { return m_shapeId; }
#endif // _DEBUG
};

//---------------------------------------------------------------------------------------------------------------------
// Circle Shape
//---------------------------------------------------------------------------------------------------------------------
class Circle : public Shape
{
  friend class Shape;
protected:
  Circle(Scene* scene, CircleDesc* desc) : Shape(scene, &m_circleDesc)
  {
    m_circleDesc = *desc;
    // store some convenient value
    m_radius = m_circleDesc.radius;
  }

  CircleDesc m_circleDesc;
  bse::Real m_radius;
public:
  const bse::Real getRadius() const { return m_radius; }
  void setRadius(const bse::Real radius)
  {
    m_radius = radius;
    setChanged(); // signal that something has changed, needs to rebuild the bb
  }

  virtual bool contains(const bse::Vec2& point) const;
protected:
  virtual void doUpdateAABB();
};

//---------------------------------------------------------------------------------------------------------------------
// Box Shape
//---------------------------------------------------------------------------------------------------------------------
class Box : public Shape
{
  friend class Shape;
protected:
  Box(Scene* scene, BoxDesc* desc) : Shape(scene, &m_boxDesc)
  {
    m_boxDesc = *desc;
    setDims(m_boxDesc.dims);
  }

  BoxDesc m_boxDesc;
  bse::Vec2 m_dims;
  bse::Vec2 m_halfDims;

public:
  void setWidth(bse::Real width)
  {
    m_dims.x = width;
    m_halfDims.x = width/2.0f;
    setChanged();
  }

  void setHeight(bse::Real height)
  {
    m_dims.y = height;
    m_halfDims.y = height/2.0f;
    setChanged();
  }

  void setDims(const bse::Vec2& dims)
  {
    m_dims = dims;
    m_halfDims = m_dims / 2.0f;
    setChanged();
  }

  bse::Vec2 getEdge(int id) const;
  void getEdgePoints(int id, bse::Vec2& start, bse::Vec2& end) const;
  bse::Vec2 getDims() const { return m_dims; }
  bse::Vec2 getHalfDims() const { return m_halfDims; }
  virtual bool contains(const bse::Vec2& point) const;
protected:
  virtual void doUpdateAABB();
};

//---------------------------------------------------------------------------------------------------------------------
// Polygon Shape
//---------------------------------------------------------------------------------------------------------------------
class Polygon : public Shape
{
  friend class Shape;
protected:
  Polygon(Scene* scene, PolygonDesc* desc) : Shape(scene, &m_polygonDesc)
  {
    m_polygonDesc = *desc;
    m_vertexes = m_polygonDesc.vertList;
  }

  PolygonDesc m_polygonDesc;
  bseVertexesList m_vertexes;
public:
  const bseVertexesList& getVertexesList() const { return m_vertexes; }
  virtual bool contains(const bse::Vec2& point) const;
protected:
  virtual void doUpdateAABB();
};

}
}

#endif // _BSE_SHAPE_INCLUDED
