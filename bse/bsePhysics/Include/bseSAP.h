#ifndef _BSE_SAP_H_INCLUDED
#define _BSE_SAP_H_INCLUDED

#include "bseCommons.h"
#include "bseTypes.h"
#include "bseAABB.h"
#include "bsePools.h"

#include "bsePhysicsTypes.h"

#include <vector>
#include <map>

namespace bse
{
namespace phx
{

class Shape;
class Scene;

//---------------------------------------------------------------------------------------------------------------------
class ShapesPair
{
public:
  ShapesPair() :
    shape1(0),
    shape2(0)
  {
  }

  ShapesPair(const Shape* s1, const Shape* s2) :
    shape1(s1),
    shape2(s2)
  {
  }

  const Shape* shape1;
  const Shape* shape2;
    // this is in general very dangerous, but for this specific task
    // it's ok, because the list is used only to collect data that later
    // are used sequentially (eg: i don't store aliases to the single elements!!!)
  static inline void swap(ShapesPair& pairA, ShapesPair& pairB)
  {
    const Shape* tempShape;
    // first shape
    tempShape = pairA.shape1;
    pairA.shape1 = pairB.shape1;
    pairB.shape1 = tempShape;

    // second shape
    tempShape = pairA.shape2;
    pairA.shape2 = pairB.shape2;
    pairB.shape2 = tempShape;
  }
  static inline void copy(ShapesPair& dest, const ShapesPair& source)
  {
    dest.shape1 = source.shape1;
    dest.shape2 = source.shape2;
  }

  bool operator<(const ShapesPair& pair)
  {
    if (shape1 < pair.shape1) return true;
    if (shape1 == pair.shape1) return (shape2 < pair.shape2);
    return false;
  }
};

// binary operator for pairs comparison
bool operator<(const ShapesPair& pair1, const ShapesPair& pair2);

typedef std::vector<ShapesPair*> ShapesPairsList;

//---------------------------------------------------------------------------------------------------------------------
class ShapePairsPool : public ShapesPairsList
{
public:
  ShapePairsPool()
  {
    resetShapePairsPool();
  }

  ~ShapePairsPool()
  {
  }

  ShapesPair* getShapesPairFromPool()
  {
    ShapesPair* pair = new (m_pairsPool.getObject()) ShapesPair();
    push_back(pair);
    return pair;
  }

  void sendShapesPairBackToPool(const bse::UInt pairIndex)
  {
    BSE_ASSERT(pairIndex<size());

    // if it's not the last one, fast removal: take the last one and put it in place of tha pairIndex one
    if (pairIndex < size()-1)
    {
      // just copy the last one in the pairIndex one!
      ShapesPair::copy(*at(pairIndex), *back());
    }

    pop_back();
  }

  void resetShapePairsPool()
  {
    clear();
    m_pairsPool.clear();
  }

  const ShapesPair* getComputedPair(const bse::UInt pairIndex) const
  {
    BSE_ASSERT(pairIndex<(bse::UInt)size());
    return at(pairIndex);
  }

  const bse::UInt getNumberOfPairs() const { return (bse::UInt)size(); }

protected:
  FrameObjectsPool<ShapesPair, 1024> m_pairsPool;
};

//---------------------------------------------------------------------------------------------------------------------
// An Interval is made by two delimiters, the begin and the end of the interval.
class Delimiter
{
public:
  Delimiter()
  {
  }

  Delimiter(bool isBeg, float val, Shape* sh) :
    isBegin(isBeg),
    value(val),
    shape(sh),
    index(-1)
  {
  }

  bool isBegin; // otherwise it's an end
  float value;
  Shape* shape; // TODO: bad design? I need the shape pointer, later when I test for overlapping
  int index;
};

//---------------------------------------------------------------------------------------------------------------------
class Interval
{
public:
  Interval() :
    lower(0),
    upper(0)
  {
  }

  Interval(Shape* shape, float l, float u)
  {
    // TODO maybe these two don't need to be on the heap? This way I would save 4 new calls per shape?
    lower = new Delimiter(true, l, shape);
    upper = new Delimiter(false, u, shape);
  }

  ~Interval()
  {
    delete lower;
    delete upper;
  }

  Delimiter* lower;
  Delimiter* upper;
};

typedef std::vector<Delimiter*> DelimitersList;
typedef std::pair<Interval*, Interval*> ShapeIntervalsPair;
typedef std::map<const Shape*, ShapeIntervalsPair> ShapeToIntervalsMap;

//---------------------------------------------------------------------------------------------------------------------
// SaP implementation
class SAP
{
public:
  SAP(Scene* scene);
  ~SAP();

  // the Pool is a I/O param. It's used to avoid allocation (preallocated pairs) and to return the candidate pairs
  void updateSAP(ShapesList* shapesList, ShapePairsPool* pool);

  void clear();

  bool intersect(const bse::Vec2& point, ShapesList& shapes);

  // Fill the output shapes vector with all the intersecting shapes.
  bool quadSearch(const bse::Vec2& point, int firstX, int firstY, int lastX, int lastY, ShapesList& outShapes);

  void addShape(Shape* newShape);

  bool removeShape(const Shape* shape);

protected:
  ShapesList*     m_shapesList;
  ShapePairsPool* m_shapePairsPool;

  DelimitersList  m_xDelimiters;
  DelimitersList  m_yDelimiters;

  Scene*         m_scene;

  ShapeToIntervalsMap m_shapeToIntervalsMap;

protected:
  void removeInterval(Interval* interval, DelimitersList* delimitersList);

  void orderedInsert(
    bool isX,                  // true if this is X axis, false otherwise
    const Shape* const shape,
    Delimiter* const lower,
    Delimiter* const upper,
    float low,
    float high,
    DelimitersList* delimiters);

  // This is maintained incrementally!
  class OverlapStatus
  {
  public:
    OverlapStatus() :
        x(false),
        y(false)
    {
    }

    OverlapStatus(bool onX, bool onY) :
        x(onX),
        y(onY)
    {
    }

    bool x;
    bool y;
  };

  typedef std::map<ShapesPair, OverlapStatus> OverlappingPairsMap;
  OverlappingPairsMap m_overlappingPairs;

  void startOverlapping(const Shape* shape1, const Shape* shape2, bool isX);
  void stopOverlapping(const Shape* shape1, const Shape* shape2, bool isX);
};

}
}

#endif // _BSE_SAP_H_INCLUDED
