#ifndef _BSE_MATH_H_INCLUDED
#define _BSE_MATH_H_INCLUDED

#include <math.h>
#include <float.h>
#include <stdlib.h>

#include "bseTypes.h"
#include <math.h>

const bse::Real bse_pi = 3.14159265359f;


//---------------------------------------------------------------------------------------------------------------------
inline bool bseIsValid(float x)
{
  return isfinite(x) != 0;
}

namespace bse
{

//---------------------------------------------------------------------------------------------------------------------
class Vec2
{
public:
  Vec2() {}
  Vec2(bse::Real x, bse::Real y) : x(x), y(y) {}
  Vec2(bse::Real v) : x(v), y(v) {}

  void reset() { x = 0.0f; y = 0.0f; }
  void set(bse::Real vx, bse::Real vy) { x = vx; y = vy; }
  void Set(bse::Real vx, bse::Real vy) { set(vx, vy); }

  Vec2 operator -() { bse::Vec2 v; v.set(-x, -y); return v; }

  static Vec2 build(bse::Real vx, bse::Real vy)
  {
    Vec2 v;
    v.set(vx, vy);
    return v;
  }

  Vec2& operator += (const Vec2& v)
  {
    x += v.x; y += v.y;
    return (*this);
  }

  Vec2& operator -= (const bse::Vec2& v)
  {
    x -= v.x; y -= v.y;
    return (*this);
  }

  Vec2& operator *= (const bse::Real a)
  {
    x *= a; y *= a;
    return (*this);
  }

  Vec2& operator /= (const Real a)
  {
    x /= a; y /= a;
    return (*this);
  }

  Vec2 operator * (const Real a) const
  {
    return Vec2(x * a, y * a);
  }

  Vec2 operator / (const Real a) const
  {
    return Vec2(x / a, y / a);
  }

  Real mag() const
  {
    return sqrtf(x * x + y * y);
  }

  Real sqrmag() const
  {
    return x * x + y * y;
  }

  Real normalize()
  {
    Real length = mag();
    if (length < FLT_EPSILON)
    {
      return 0.0f;
    }

    Real invLength = 1.0f / length;
    x *= invLength;
    y *= invLength;

    return length;
  }

  bool isValid() const
  {
    return bseIsValid(x) && bseIsValid(y);
  }

  Real x, y;

  static const Vec2 XAxis;
  static const Vec2 YAxis;
  static const Vec2 One;
  static const Vec2 Zero;
};

//---------------------------------------------------------------------------------------------------------------------
class Mat22
{
public:
  Mat22() { setIdentity(); }
  Mat22(const Vec2& c1, const Vec2& c2)
  {
    col1 = c1;
    col2 = c2;
  }

  Mat22(Real angle)
  {
    Real c = cosf(angle), s = sinf(angle);
    col1.x = c; col2.x = -s;
    col1.y = s; col2.y = c;
  }

  void set(const Vec2& c1, const Vec2& c2)
  {
    col1 = c1;
    col2 = c2;
  }

  void set(Real angle)
  {
    Real c = cosf(angle), s = sinf(angle);
    col1.x = c; col2.x = -s;
    col1.y = s; col2.y = c;
  }

  void setIdentity()
  {
    col1.x = 1.0f; col2.x = 0.0f;
    col1.y = 0.0f; col2.y = 1.0f;
  }

  void reset()
  {
    col1.x = 0.0f; col2.x = 0.0f;
    col1.y = 0.0f; col2.y = 0.0f;
  }

  Mat22 invert() const
  {
    Real a = col1.x, b = col2.x, c = col1.y, d = col2.y;
    Mat22 B;
    Real det = a * d - b * c;
    //bseAssert(det != 0.0f);
    det = 1.0f / det;
    B.col1.x =  det * d;	B.col2.x = -det * b;
    B.col1.y = -det * c;	B.col2.y =  det * a;
    return B;
  }

  Mat22 transpose() const
  {
    Mat22 B;
    B.col1.x =  col1.x;
    B.col1.y =  col2.x;
    B.col2.x =  col1.y;
    B.col2.y =  col2.y;
    return B;
  }

  void rotate(const Real& angle)
  {
    Mat22 rot(angle);
    Mat22 temp = *this;
    temp = temp.mul(rot);
    *this = temp;
  }

  inline Mat22 mul(const Mat22& B) const
  {
    Mat22 C;
    C.set((*this).mul(B.col1), (*this).mul(B.col2));
    return C;
  }

  Vec2 mul(const Vec2& v) const
  {
    Vec2 u;
    u.set(col1.x * v.x + col2.x * v.y, col1.y * v.x + col2.y * v.y);
    return u;
  }

  Mat22 mul(const Real& x) const
  {
    Mat22 u;
    u.set(col1 * x , col2 * x);
    return u;
  }

  Real toAngle() const
  {
    Real x = (Real)acos(col1.x);
    Real angle = x;
    if (col1.y<0)
      angle = -angle;
    return angle;
  }

  Vec2 col1, col2;
};

} // namespace bse

//---------------------------------------------------------------------------------------------------------------------
inline bse::Real bseDot(const bse::Vec2& a, const bse::Vec2& b)
{
  return a.x * b.x + a.y * b.y;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Real bseCross(const bse::Vec2& a, const bse::Vec2& b)
{
  return a.x * b.y - a.y * b.x;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseCross(const bse::Vec2& a, bse::Real s)
{
  bse::Vec2 v; v.set(s * a.y, -s * a.x);
  return v;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseCross(bse::Real s, const bse::Vec2& a)
{
  bse::Vec2 v; v.set(-s * a.y, s * a.x);
  return v;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseMul(const bse::Mat22& A, const bse::Vec2& v)
{
  bse::Vec2 u;
  u.set(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
  return u;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseMulT(const bse::Mat22& A, const bse::Vec2& v)
{
  bse::Vec2 u;
  u.set(bseDot(v, A.col1), bseDot(v, A.col2));
  return u;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 operator + (const bse::Vec2& a, const bse::Vec2& b)
{
  bse::Vec2 v;
  v.set(a.x + b.x, a.y + b.y);
  return v;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 operator - (const bse::Vec2& a, const bse::Vec2& b)
{
  bse::Vec2 v; v.set(a.x - b.x, a.y - b.y);
  return v;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 operator * (bse::Real s, const bse::Vec2& a)
{
  bse::Vec2 v;
  v.set(s * a.x, s * a.y);
  return v;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Mat22 operator + (const bse::Mat22& A, const bse::Mat22& B)
{
  bse::Mat22 C;
  C.set(A.col1 + B.col1, A.col2 + B.col2);
  return C;
}

//---------------------------------------------------------------------------------------------------------------------
// A^T * B
inline bse::Mat22 bseMulT(const bse::Mat22& A, const bse::Mat22& B)
{
  bse::Vec2 c1;
  c1.set(bseDot(A.col1, B.col1), bseDot(A.col2, B.col1));
  bse::Vec2 c2;
  c2.set(bseDot(A.col1, B.col2), bseDot(A.col2, B.col2));
  bse::Mat22 C;
  C.set(c1, c2);
  return C;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Real bseAbs(bse::Real a)
{
  return a > 0.0f ? a : -a;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseAbs(const bse::Vec2& a)
{
  bse::Vec2 b; b.set(fabsf(a.x), fabsf(a.y));
  return b;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Mat22 bseAbs(const bse::Mat22& A)
{
  bse::Mat22 B;
  B.set(bseAbs(A.col1), bseAbs(A.col2));
  return B;
}

//---------------------------------------------------------------------------------------------------------------------
template <typename T>
inline T bseMin(T a, T b)
{
  return a < b ? a : b;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseMin(const bse::Vec2& a, const bse::Vec2& b)
{
  bse::Vec2 c;
  c.x = bseMin(a.x, b.x);
  c.y = bseMin(a.y, b.y);
  return c;
}

//---------------------------------------------------------------------------------------------------------------------
template <typename T>
inline T bseMax(T a, T b)
{
  return a > b ? a : b;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseMax(const bse::Vec2& a, const bse::Vec2& b)
{
  bse::Vec2 c;
  c.x = bseMax(a.x, b.x);
  c.y = bseMax(a.y, b.y);
  return c;
}

//---------------------------------------------------------------------------------------------------------------------
template <typename T>
inline T bseClamp(T a, T low, T high)
{
  return bseMax(low, bseMin(a, high));
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Vec2 bseClamp(const bse::Vec2& a, const bse::Vec2& low, const bse::Vec2& high)
{
  return bseMax(low, bseMin(a, high));
}

//---------------------------------------------------------------------------------------------------------------------
template<typename T> inline void bseSwap(T& a, T& b)
{
  T tmp = a;
  a = b;
  b = tmp;
}

//---------------------------------------------------------------------------------------------------------------------
inline float bseSign(float x)
{
  return (x<0.0f?-1.0f:1.0f);
}

//---------------------------------------------------------------------------------------------------------------------
// bseRandom number in range [-1,1]
inline bse::Real bseRandom()
{
  bse::Real r = (bse::Real)rand();
  r /= RAND_MAX;
  r = 2.0f * r - 1.0f;
  return r;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Real bseRandom(bse::Real lo, bse::Real hi)
{
  bse::Real r = (bse::Real)rand();
  r /= RAND_MAX;
  r = (hi - lo) * r + lo;
  return r;
}

//---------------------------------------------------------------------------------------------------------------------
inline bse::Int bseRandom(bse::Int lo, bse::Int hi)
{
  bse::Real r = bseRandom((bse::Real)lo, (bse::Real)hi);
  return static_cast<bse::Int>(r);
}

#endif // _BSE_MATH_H_INCLUDED
