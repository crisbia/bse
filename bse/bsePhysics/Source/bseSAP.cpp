#include "bseSAP.h"
#include "bseShape.h"
#include "bseScene.h"
#include "bseProfilingTools.h"

// Define this if you want some (slow) sanity check on the intervals and on the overlapping shapes to be performed.
#ifdef _DEBUG
//#define SAP_SANITY_CHECK
#endif

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
bool operator<(const ShapesPair& pair1, const ShapesPair& pair2)
{
  if (pair1.shape1 < pair2.shape1)
    return true;
  if (pair1.shape1 == pair2.shape1)
    return (pair1.shape2 < pair2.shape2);
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::removeInterval(Interval* interval, DelimitersList* delimitersList)
{
  // This is an expensive operation as when a delimiter is removed all the following ones have to be shifted and their indexes updated.
  // TODO: To minimize the cost, treat the lower and the upper delimiters together so that there is less stuff to shift back.
  BSE_ASSERT(interval && delimitersList);
  Delimiter* lower = interval->lower;
  Delimiter* upper = interval->upper;
  if (lower->index != -1 && upper->index != -1)
  {
    BSE_ASSERT(lower->index < (int)delimitersList->size() && upper->index < (int)delimitersList->size());
    for (size_t i=lower->index; i<delimitersList->size()-1; ++i)
    {
      (*delimitersList)[i] = (*delimitersList)[i+1];
      (*delimitersList)[i]->index = i;
    }
    delimitersList->pop_back();

    for (size_t i=upper->index; i<delimitersList->size()-1; ++i)
    {
      (*delimitersList)[i] = (*delimitersList)[i+1];
      (*delimitersList)[i]->index = i;
    }
    delimitersList->pop_back();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::orderedInsert(
  bool isX,                  // true if this is X axis, false otherwise
  const Shape* const shape,
  Delimiter* const inShapeLower,
  Delimiter* const inShapeUpper,
  float low,
  float high,
  DelimitersList* delimiters)
{
  int startIndexLow = 0, startIndexHigh = 0;
  int lowI = inShapeLower->index;
  int highI = inShapeUpper->index;

  // if the indexes are -1, it means that the bb has not been considered yet (eg: new insertion? first time?)
  if (lowI==-1 || highI==-1) // nb: if one is -1, the other is -1 as well
  {
    // put them in the back for convenience
    delimiters->push_back(inShapeLower);
    startIndexLow = static_cast<int>(delimiters->size()) - 1;
    delimiters->push_back(inShapeUpper);
    startIndexHigh = static_cast<int>(delimiters->size()) - 1;
  }
  else
  {
    // intervals already exist, start look up from the old index
    startIndexLow = lowI;
    startIndexHigh = highI;
    // update the values, the shape has moved!
    (*delimiters)[lowI]->value = low;
    (*delimiters)[highI]->value = high;
  }

  // Now we need to make the list ordered.
  int startIndex[2] = {startIndexLow, startIndexHigh};
  int finalIndex[2] = {startIndexLow, startIndexHigh};  // this is going to store my indexes, always!
  Delimiter* myDelimiters[2] = {inShapeLower, inShapeUpper};
  float refVal[2] = {low, high};
  int start, i;

  for (int iteration=0; iteration<2; ++iteration)
  {
    Delimiter* myDeli = myDelimiters[iteration];
    {
      // backward
      start = startIndex[iteration] - 1;
      i = start;
      while (i>=0)
      {
        Delimiter* delimiter = (*delimiters)[i];
        if (delimiter->value > refVal[iteration])
        {
          // Need to swap to fix the order.
          {
            Delimiter* temp = (*delimiters)[i];
            (*delimiters)[i] = (*delimiters)[finalIndex[iteration]];
            (*delimiters)[finalIndex[iteration]] = temp;
          }

          Shape* const otherShape = delimiter->shape;

          // Avoid registering overlapping for static-static.
          BSE_ASSERT(shape != otherShape);
          if (!(shape->isStatic() && otherShape->isStatic()))
          {
            if (myDeli->isBegin && !delimiter->isBegin)
            {
              startOverlapping(shape, otherShape, isX);
            }
            else if (!myDeli->isBegin && delimiter->isBegin)
            {
              stopOverlapping(shape, otherShape, isX);
            }
          }

          BSE_ASSERT(otherShape);
          ShapeToIntervalsMap::iterator iter = m_shapeToIntervalsMap.find(otherShape);
          BSE_ASSERT(iter != m_shapeToIntervalsMap.end());
          ShapeIntervalsPair& intervalsPair = iter->second;
          BSE_ASSERT(intervalsPair.first && intervalsPair.second);
          Interval* const shapeInterval = isX ? intervalsPair.first : intervalsPair.second;
          Delimiter* const shapeLower = shapeInterval->lower;
          Delimiter* const shapeUpper = shapeInterval->upper;

          if (shapeLower->index == i)
          {
            shapeLower->index = finalIndex[iteration];
          }
          else
          {
            shapeUpper->index = finalIndex[iteration];
          }

          // new index for the low interval
          finalIndex[iteration] = i;
        }
        else
        {
          break;
        }
        --i;
      }
    }

    if (finalIndex[iteration] == startIndex[iteration]) // I didn't move, it makes sense to try the other direction
    {
      // forward
      start = startIndex[iteration] + 1;
      i = start;
      while (i<static_cast<int>(delimiters->size()))
      {
        Delimiter* delimiter = (*delimiters)[i];
        if (delimiter->value < refVal[iteration])
        {
          // Need to swap to fix the order.
          {
            Delimiter* const temp = (*delimiters)[i];
            (*delimiters)[i] = (*delimiters)[finalIndex[iteration]];
            (*delimiters)[finalIndex[iteration]] = temp;
          }

          Shape* const otherShape = delimiter->shape;
          BSE_ASSERT(otherShape);

          BSE_ASSERT(shape != otherShape);

          // Avoid recording the overlapping for static-static.
          if (!(shape->isStatic() && otherShape->isStatic()))
          {
            if (myDeli->isBegin && !delimiter->isBegin)
            {
              stopOverlapping(shape, otherShape, isX);
            }
            else if (!myDeli->isBegin && delimiter->isBegin)
            {
              startOverlapping(shape, otherShape, isX);
            }
          }

          ShapeToIntervalsMap::iterator iter = m_shapeToIntervalsMap.find(otherShape);
          BSE_ASSERT(iter != m_shapeToIntervalsMap.end());
          ShapeIntervalsPair& intervalsPair = iter->second;
          BSE_ASSERT(intervalsPair.first && intervalsPair.second);
          Interval* const shapeInterval = isX ? intervalsPair.first : intervalsPair.second;
          Delimiter* const shapeLower = shapeInterval->lower;
          Delimiter* const shapeUpper = shapeInterval->upper;

          if (shapeLower->index == i)
          {
            shapeLower->index = finalIndex[iteration];
          }
          else
          {
            shapeUpper->index = finalIndex[iteration];
          }

          // new index for the low interval
          finalIndex[iteration] = i;
        }
        else
        {
          break;
        }
        ++i;
      }
    }
  }

  // Now the intervals are in the right place. I can output the right index for the bb.
  inShapeLower->index = finalIndex[0];
  inShapeUpper->index = finalIndex[1];

#ifdef SAP_SANITY_CHECK
  //////////////////// sanity check, to be removed: check if the interval lists are ordered
  {
    for (bse::UInt intervalIndex=0; intervalIndex<delimiters->size()-1; ++intervalIndex)
    {
      if ((*delimiters)[intervalIndex]->value > (*delimiters)[intervalIndex+1]->value)
      {
        printf("Warning: interval %d not in order\n", intervalIndex);
      }
    }
  }
#endif
}

//---------------------------------------------------------------------------------------------------------------------
SAP::SAP(Scene* scene) :
  m_scene(scene)
{
}

//---------------------------------------------------------------------------------------------------------------------
SAP::~SAP()
{
  clear();
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::clear()
{
  m_xDelimiters.clear();
  m_yDelimiters.clear();

  // The shape to interval map owns the intervals. Clean up before clearing the map.
  for (ShapeToIntervalsMap::iterator iter = m_shapeToIntervalsMap.begin(); iter != m_shapeToIntervalsMap.end(); ++iter)
  {
    ShapeIntervalsPair& intervalsPair = iter->second;
    delete intervalsPair.first;
    delete intervalsPair.second;
  }
  m_shapeToIntervalsMap.clear();
  m_overlappingPairs.clear();
}

//---------------------------------------------------------------------------------------------------------------------
bool SAP::intersect(const bse::Vec2& point, ShapesList& outShapes)
{
  // Start the recursion with all the current intervals.
  return quadSearch(point, 0, 0, (int)m_xDelimiters.size()-1, (int)m_yDelimiters.size()-1, outShapes);
}

//---------------------------------------------------------------------------------------------------------------------
bool SAP::quadSearch(
    const bse::Vec2& point,
    int firstX, int firstY,
    int lastX, int lastY,
    ShapesList& outShapes)
{
  const int minDiff = 5;

  bseClamp(firstX, 0, (int)m_xDelimiters.size()-1);
  bseClamp(lastX, 0, (int)m_xDelimiters.size()-1);
  bseClamp(firstY, 0, (int)m_yDelimiters.size()-1);
  bseClamp(lastY, 0, (int)m_yDelimiters.size()-1);

  // Try to intersect the quadrant. If the point is inside, go into.
  Delimiter* xFirst = m_xDelimiters[firstX];
  Delimiter* xLast  = m_xDelimiters[lastX];
  Delimiter* yFirst = m_yDelimiters[firstY];
  Delimiter* yLast  = m_yDelimiters[lastY];

  if (!(point.x >= xFirst->value && point.x <= xLast->value && point.y >= yFirst->value && point.y <= yLast->value))
    return false;

  if (lastX-firstX<=minDiff)
  {
    // Stop the recursion, collect intersecting shapes.
    int numShapes = 0;
    for (int i = firstX; i<=lastX; ++i)
    {
      if (m_xDelimiters[i]->shape->getAABB()->contains(point))
      {
        outShapes.push_back(m_xDelimiters[i]->shape);
        ++numShapes;
      }
    }

    return numShapes>0;
  }

  // Recursion. Split in 4 quadrants and perform the quadsearch on each one. Because we want to accumulate all
  // the possible shapes, we have to perform the full search.
  int mX = (firstX + lastX) / 2;
  int mY = (firstY + lastY) / 2;
  bool q1 = quadSearch(point, firstX, firstY, mX, mY, outShapes);
  bool q2 = quadSearch(point, mX, firstY, lastX, mY, outShapes);
  bool q3 = quadSearch(point, mX, mY, lastX, lastY, outShapes);
  bool q4 = quadSearch(point, firstX, mY, mX, lastY, outShapes);

  return q1 || q2 || q3 || q4;
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::updateSAP(ShapesList* shapesList, ShapePairsPool* pool)
{
  const bse::UInt numberOfShapes = static_cast<bse::UInt>(shapesList->size());
  if (numberOfShapes == 0)
  {
    return;
  }

  m_shapesList = shapesList;
  m_shapePairsPool = pool;
  pool->resetShapePairsPool();

  // First pass, update the AABB
  {
#ifdef BSE_ENABLE_PROFILER
    bse::ProfileTaskWrapper updateOrderedIntervals(m_scene->getProfiler(), "BSE_UPDATE_ORDERED_INTERVALS");
#endif // BSE_ENABLE_PROFILER

    for (ShapesList::const_iterator iter = shapesList->begin(); iter != shapesList->end(); ++iter)
    {
      Shape* const shape = (*iter);
      if (shape->hasChanged() || shape->hasMoved())
      {
        {
#ifdef BSE_ENABLE_PROFILER
          bse::ProfileTaskWrapper updateAABB(m_scene->getProfiler(), "BSE_UPDATE_AABBS");
#endif // BSE_ENABLE_PROFILER
          shape->updateAABB();
        }

        // ordered insertion of the interval list
        ShapeToIntervalsMap::iterator shapesMapIter = m_shapeToIntervalsMap.find(shape);
        BSE_ASSERT(shapesMapIter != m_shapeToIntervalsMap.end()); // Shape has to be part of the map!!!
        const ShapeIntervalsPair& intervalsPair = shapesMapIter->second;
        Interval* xInterval = intervalsPair.first;
        Interval* yInterval = intervalsPair.second;
        BSE_ASSERT(xInterval && yInterval && xInterval->lower && xInterval->upper && yInterval->lower && yInterval->upper);

        {
#ifdef BSE_ENABLE_PROFILER
          bse::ProfileTaskWrapper updateIntervals(m_scene->getProfiler(), "BSE_UPDATE_INTERVALS");
#endif // BSE_ENABLE_PROFILER
          const AABB* const aabb = shape->getAABB();
          orderedInsert(true, shape, xInterval->lower, xInterval->upper, aabb->low.x, aabb->high.x, &m_xDelimiters);
          orderedInsert(false, shape, yInterval->lower, yInterval->upper, aabb->low.y, aabb->high.y, &m_yDelimiters);
        }
      }
    }
  }

//////////////////// sanity, check if the interval lists are ordered
#ifdef SAP_SANITY_CHECK
  {
    for (bse::UInt intervalIndex=0; intervalIndex < m_xDelimiters.size()-1; ++intervalIndex)
    {
      if (m_xDelimiters[intervalIndex]->value > m_xDelimiters[intervalIndex+1]->value)
      {
        printf("Warning: x-interval %d not in the right order\n", intervalIndex);
      }

      if (m_yDelimiters[intervalIndex]->value > m_yDelimiters[intervalIndex+1]->value)
      {
        printf("Warning: y-interval %d not in the right order\n", intervalIndex);
      }
    }
  }
#endif

  {
#ifdef BSE_ENABLE_PROFILER
    bse::ProfileTaskWrapper copyingFinalResults(m_scene->getProfiler(), "BSE_COPY_OVERLAPPING_PAIRS");
#endif // BSE_ENABLE_PROFILER

    for (SAP::OverlappingPairsMap::const_iterator iter = m_overlappingPairs.begin(); iter != m_overlappingPairs.end(); ++iter)
    {
      const ShapesPair& overlappingPair = iter->first;
      const OverlapStatus& overlappingStatus = iter->second;
      if (overlappingStatus.x && overlappingStatus.y)
      {
        ShapesPair* const outPair = m_shapePairsPool->getShapesPairFromPool();
        outPair->shape1 = overlappingPair.shape1;
        outPair->shape2 = overlappingPair.shape2;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::addShape(Shape* newShape)
{
  newShape->updateAABB();
  const AABB* aabb = newShape->getAABB();
  ShapeIntervalsPair intervalsPair;
  intervalsPair.first = new Interval(newShape, aabb->low.x, aabb->high.x);
  intervalsPair.second = new Interval(newShape, aabb->low.y, aabb->high.y);
  m_shapeToIntervalsMap.insert(std::make_pair((const Shape*)newShape, intervalsPair));

  // Ordered insertion of the interval list
  orderedInsert(true, newShape, intervalsPair.first->lower, intervalsPair.first->upper, aabb->low.x, aabb->high.x, &m_xDelimiters);
  orderedInsert(false, newShape, intervalsPair.second->lower, intervalsPair.second->upper, aabb->low.y, aabb->high.y, &m_yDelimiters);
}

//---------------------------------------------------------------------------------------------------------------------
bool SAP::removeShape(const Shape* shape)
{
  ShapeToIntervalsMap::iterator shapeIter = m_shapeToIntervalsMap.find(shape);
  if (shapeIter != m_shapeToIntervalsMap.end())
  {
    ShapeIntervalsPair& intervalsPair = shapeIter->second;

    // Expensive operation. Remove the delimiters from the ordered lists.
    Interval* xInterval = intervalsPair.first;
    Interval* yInterval = intervalsPair.second;
    removeInterval(xInterval, &m_xDelimiters);
    removeInterval(yInterval, &m_yDelimiters);

    delete intervalsPair.first;
    delete intervalsPair.second;

    m_shapeToIntervalsMap.erase(shapeIter);

    // Another expensive operation is to remove the overlaps where this shape is involved.
    SAP::OverlappingPairsMap::iterator iter = m_overlappingPairs.begin();
    while (iter != m_overlappingPairs.end())
    {
      const ShapesPair& overlappingPair = iter->first;
      if (overlappingPair.shape1 == shape || overlappingPair.shape2 == shape)
      {
        m_overlappingPairs.erase(iter++);
      }
      else
      {
       ++iter;
      }
    }

    // TODO This should make better usage of the SAP properties. For example there should be a way of moving the shape
    // very 'far away' and perform a sap update that would take care of removing the overlaps where the shape is involved.
    return true;
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::startOverlapping(const Shape* shape1, const Shape* shape2, bool isX)
{
  ShapesPair sp(shape1, shape2);
  if (shape1 > shape2)
  {
    sp = ShapesPair(shape2, shape1);
  }

  OverlappingPairsMap::iterator iter = m_overlappingPairs.find(sp);
  if (iter != m_overlappingPairs.end())
  {
    if (isX)
      iter->second.x = true;
    else
      iter->second.y = true;
  }
  else
  {
    if (isX)
      m_overlappingPairs.insert(std::make_pair(sp, OverlapStatus(true, false)));
    else
      m_overlappingPairs.insert(std::make_pair(sp, OverlapStatus(false, true)));
  }
}

//---------------------------------------------------------------------------------------------------------------------
void SAP::stopOverlapping(const Shape* shape1, const Shape* shape2, bool isX)
{
  ShapesPair sp(shape1, shape2);
  if (shape1 > shape2)
  {
    sp = ShapesPair(shape2, shape1);
  }

  OverlappingPairsMap::iterator iter = m_overlappingPairs.find(sp);
  if (iter != m_overlappingPairs.end())
  {
    if (isX)
      iter->second.x = false;
    else
      iter->second.y = false;

    // Remove the record if not overlapping on both axis.
    if (!iter->second.x && !iter->second.y)
    {
      m_overlappingPairs.erase(iter);
    }
  }
}

} // namespace phx
} // namespace bse
