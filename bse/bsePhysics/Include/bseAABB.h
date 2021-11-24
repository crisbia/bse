#ifndef _BSE_AABB_H_INCLUDED
#define _BSE_AABB_H_INCLUDED

#include "bseMath.h"

namespace bse
{
namespace phx
{
//---------------------------------------------------------------------------------------------------------------------
class AABB
{
public:
  AABB() : 
    low(0,0), 
    high(0,0)
  { 
  }

  AABB(const bse::Vec2& l, const bse::Vec2& h) : 
    low(l), 
    high(h)
  {
  }

  void enlarge(float d)
  {
    low -= (bse::Vec2::One * d);
    high += (bse::Vec2::One * d);
  }

  void enclose(const AABB& toEncl)
  {
    // low extent
    low.x = bseMin(low.x, toEncl.low.x);
    low.y = bseMin(low.y, toEncl.low.y);

    high.x = bseMax(high.x, toEncl.high.x);
    high.y = bseMax(high.y, toEncl.high.y);
  }

  void enclose(const bse::Vec2& pointToEncl)
  {
    low.x = bseMin(low.x, pointToEncl.x);
    low.y = bseMin(low.y, pointToEncl.y);

    high.x = bseMax(high.x, pointToEncl.x);
    high.y = bseMax(high.y, pointToEncl.y);
  }

  bool contains(const bse::Vec2& point) const
  {
    if (point.x < low.x || point.y < low.y || point.x > high.x || point.y > high.y)
      return false;
    return true;
  }

  bool intersect(const AABB* aabb) const
  {
    // A = this, B = other

    // intersection (it should cover the containment as well)
    if (high.x >= aabb->low.x && low.x <= aabb->high.x && high.y >= aabb->low.y && low.y <= aabb->high.y)
      return true;

    if (aabb->low.x >= high.x && aabb->high.x <= low.x && aabb->low.y >= high.y && aabb->high.y <= low.y)
      return true;

    return false;
  }

  bse::Vec2 low;
  bse::Vec2 high;
};

}
}

#endif // _BSE_AABB_H_INCLUDED