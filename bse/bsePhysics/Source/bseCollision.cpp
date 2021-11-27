#include "bseCollision.h"
#include "bseDynamics.h"
#include "bseShape.h"

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
bool operator<(const CollisionPair& pair1, const CollisionPair& pair2)
{
  if (pair1.shape1 < pair2.shape1)
    return true;
  if (pair1.shape1 == pair2.shape1)
    return (pair1.shape2 < pair2.shape2);
  return false;

}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderShapeShape(
    const Shape* collision_shape1, const Shape* collision_shape2,
    ContactsPool* pool, CollisionData* data)
{
  Shape* shape1 = (Shape*)collision_shape1;
  Shape* shape2 = (Shape*)collision_shape2;
  if (shape1->getBody()->isStatic() && shape2->getBody()->isStatic())
    return 0;

  bseShapeType type1 = shape1->getType();
  bseShapeType type2 = shape2->getType();

  // types in order...
  if (type2 < type1)
  {
    bseShapeType temp = type1;
    type1 = type2;
    type2 = temp;

    Shape* tempShape = shape1;
    shape1 = shape2;
    shape2 = tempShape;
  }

  int numberOfContacts = 0;

  switch (type1)
  {
  case BSE_SHAPE_CIRCLE:
    {
      const Circle* circle1 = (Circle*)shape1;
      switch (type2)
      {
      case BSE_SHAPE_CIRCLE:
        {
          const Circle* circle2 = (Circle*)shape2;
          numberOfContacts = colliderCircleCircle(circle1, circle2, pool, data);
        }
        break;
      case BSE_SHAPE_BOX:
        {
          const Box* box2 = (Box*)shape2;
          numberOfContacts = colliderCircleBox(circle1, box2, pool, data);
        }
        break;
      case BSE_SHAPE_POLYGON:
        {
          const Polygon* poly2 = (Polygon*)shape2;
          numberOfContacts = colliderCirclePolygon(circle1, poly2, pool, data);
        }
        break;
      default:
        break;
      }
    }
    break;
  case BSE_SHAPE_BOX:
    {
      const Box* box1 = (Box*)shape1;
      switch (type2)
      {
      case BSE_SHAPE_BOX:
        {
          const Box* box2 = (Box*)shape2;
          numberOfContacts = colliderBoxBox(box1, box2, pool, data);
        }
        break;
      case BSE_SHAPE_POLYGON:
        {
          const Polygon* poly2 = (Polygon*)shape2;
          numberOfContacts = colliderBoxPolygon(box1, poly2, pool, data);
        }
        break;
      default:
        break;
      }
    }
    break;
  case BSE_SHAPE_POLYGON:
    {
      const Polygon* poly1 = (Polygon*)shape1;
      switch (type2)
      {
      case BSE_SHAPE_POLYGON:
        {
          const Polygon* poly2 = (Polygon*)shape2;
          numberOfContacts = colliderPolygonPolygon(poly1, poly2, pool, data);
        }
        break;
      default:
        break;
      }
    }
    break;
  default:
    break;
  }

  if (numberOfContacts>0)
  {
    // fill in the friction/restitution
    Material* material1 = shape1->getMaterial();
    Material* material2 = shape2->getMaterial();

    bse::Real combinedFriction = Material::combineFriction(material1, material2);
    bse::Real combinedRestitution = Material::combineRestitution(material1, material2);

    for (int contactIndex = 0; contactIndex<numberOfContacts; ++contactIndex)
    {
      Contact* contact = data->contacts[contactIndex];
      contact->friction = combinedFriction;
      contact->restitution = combinedRestitution;

      bse::Vec2 relPos1 = contact->contactPoint - contact->body1->getPosition();
      bse::Vec2 relPos2 = contact->contactPoint - contact->body2->getPosition();

      contact->local1 = bseMulT(contact->body1->getRotationMatrix(), relPos1);
      contact->local2 = bseMulT(contact->body2->getRotationMatrix(), relPos2);

      // Compute the velocity target
      bse::Real vRel = bseDot(
        contact->contactNormal,
        contact->body1->getLinearVelocity() + bseCross(contact->body1->getAngularVelocity(), relPos1)
        - contact->body2->getLinearVelocity() - bseCross(contact->body2->getAngularVelocity(), relPos2));

      contact->targetVelocity = -contact->restitution * vRel;

      contact->normalVelocityImpulse = 0;
      contact->tangentVelocityImpulse= 0;
      contact->positionImpulse = 0;


      bse::Vec2 temp1Norm = contact->body1->getInvInertia() * bseCross(relPos1, contact->contactNormal);
      bse::Vec2 temp2Norm = contact->body2->getInvInertia() * bseCross(relPos2, contact->contactNormal);
      contact->normalMass = 1.0f /
          (contact->body1->getInvMass() +
           contact->body2->getInvMass() +
           bseDot( contact->contactNormal, bseCross( temp1Norm, relPos1 ) + bseCross( temp2Norm, relPos2 ) ) );

      contact->separationVelocity = vRel;
    }
  }
  return numberOfContacts;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderCircleCircle(
  const Circle* circle1, const Circle* circle2,
  ContactsPool* pool, CollisionData* data)
{
  const bse::Vec2 p1 = circle1->getPosition();
  const bse::Real r1 = circle1->getRadius();
  const bse::Vec2 p2 = circle2->getPosition();
  const bse::Real r2 = circle2->getRadius();

  bse::Vec2 dir = p1 - p2;
  bse::Real d = dir.mag();
  if (d > r1+r2)
    return 0; // early out

  dir /= d;

  // "allocate" a contact from the pool
  data->contacts[0] = pool->getContactFromPool();
  // use the contact
  Contact* contact = data->contacts[0];
  contact->contactNormal = dir;
  contact->contactPoint = p1 + dir * 0.5f; // don't like this...
  contact->penetration = r1 + r2 - d;
  contact->body1 = circle1->getBody();
  contact->body2 = circle2->getBody();
  return 1;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderCircleBox(
  const Circle* circle, const Box* box,
  ContactsPool* pool, CollisionData* data)
{
  const bse::Vec2 pCircle = circle->getPosition();
  const Mat22 oCircle = circle->getRotationMatrix();
  const bse::Vec2 pBox = box->getPosition();
  const Mat22 oBox = box->getRotationMatrix();
  const bse::Real radius = circle->getRadius();
  const bse::Vec2 halfSizes = box->getHalfDims();

  bse::Vec2 relPCircle = bseMul( oBox.invert(), pCircle - pBox );

  // early out... check the two separating axis
  // if at least one sep is not overlapping, there's no collision for sure
  if (  fabs(relPCircle.x) - radius > halfSizes.x  ||
    fabs(relPCircle.y) - radius > halfSizes.y )
    return 0;

  // clamp coordinated
  bse::Real dist;
  bse::Vec2 closestPt(0,0);

  dist = relPCircle.x;
  if (dist > halfSizes.x)
    dist = halfSizes.x;
  if (dist < -halfSizes.x)
    dist = -halfSizes.x;
  closestPt.x = dist;

  dist = relPCircle.y;
  if (dist > halfSizes.y)
    dist = halfSizes.y;
  if (dist < -halfSizes.y)
    dist = -halfSizes.y;
  closestPt.y = dist;

  dist = (closestPt - relPCircle).sqrmag();
  if (dist > radius * radius)
    return 0;

  // ok, now we know that we are in contact...

  // contact point in world coordinates
    // allocate the contact from the pool
  data->contacts[0] = pool->getContactFromPool();
  Contact* contact = data->contacts[0];
  contact->contactPoint = pBox + bseMul( oBox, closestPt );
  contact->contactNormal = (pCircle - contact->contactPoint);
  contact->contactNormal.normalize();
  contact->penetration = radius - sqrt(dist);
  contact->body1 = circle->getBody();
  contact->body2 = box->getBody();

  return 1;
}

//---------------------------------------------------------------------------------------------------------------------
bse::Real transformToAxis(const Box* box, const bse::Vec2& axis)
{
  const bse::Vec2 half = box->getHalfDims();
  const Mat22 ori = box->getRotationMatrix();
  return half.x * fabs(bseDot(axis, ori.col1)) + half.y * fabs(bseDot(axis, ori.col2)) ;
}

//---------------------------------------------------------------------------------------------------------------------
bool overlapOnAxis(const Box* one, const Box* two, const bse::Vec2& axis)
{
  // project
  bse::Real onePrj = transformToAxis(one, axis);
  bse::Real twoPrj = transformToAxis(two, axis);

  // vector between centers
  bse::Vec2 distVec = two->getPosition() - one->getPosition();

  // project distVec onto the axis
  bse::Real dist = fabs(bseDot(distVec, axis));

  // check overlap
  return (dist < onePrj+twoPrj);
}

//---------------------------------------------------------------------------------------------------------------------
/*
Note: the following is courtesy of Erin Catto, www.gphysics.org
*/

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

//---------------------------------------------------------------------------------------------------------------------
enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

//---------------------------------------------------------------------------------------------------------------------
enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};


//---------------------------------------------------------------------------------------------------------------------
void flipFeature(FeaturePair& fp)
{
	bseSwap(fp.e.inEdge1, fp.e.inEdge2);
	bseSwap(fp.e.outEdge1, fp.e.outEdge2);
}

//---------------------------------------------------------------------------------------------------------------------
int bseClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
					               const bse::Vec2& normal, float offset, char clipEdge)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = bseDot(normal, vIn[0].v) - offset;
	float distance1 = bseDot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = NO_EDGE;
		}
		++numOut;
	}

	return numOut;
}

//---------------------------------------------------------------------------------------------------------------------
static void bseComputeIncidentEdge(ClipVertex c[2], const bse::Vec2& h, const bse::Vec2& pos,
								                   const Mat22& Rot, const bse::Vec2& normal)
{
	// The normal is from the reference box. Convert it
	// to the incident boxe's frame and flip sign.
  Mat22 RotT = Rot.transpose();
	bse::Vec2 n = -(bseMul(RotT, normal));
	bse::Vec2 nAbs = bseAbs(n);

	if (nAbs.x > nAbs.y)
	{
		if (bseSign(n.x) > 0.0f)
		{
			c[0].v.Set(h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE3;
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v.Set(h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		else
		{
			c[0].v.Set(-h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v.Set(-h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}
	else
	{
		if (bseSign(n.y) > 0.0f)
		{
			c[0].v.Set(h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v.Set(-h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		else
		{
			c[0].v.Set(-h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v.Set(h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;
		}
	}

	c[0].v = pos + bseMul(Rot, c[0].v);
	c[1].v = pos + bseMul(Rot, c[1].v);
}

//---------------------------------------------------------------------------------------------------------------------
// The normal points from A to B
int bseCollideBoxBox(const Box* boxA, const Box* boxB, Contact* contacts)
{
	// Setup
  bse::Vec2 a_dims = boxA->getDims();
  bse::Vec2 a_position = boxA->getPosition();

  bse::Vec2 b_dims = boxB->getDims();
  bse::Vec2 b_position = boxB->getPosition();

  bse::Vec2 hA = a_dims * 0.5f;
	bse::Vec2 hB = b_dims * 0.5f;

	bse::Vec2 posA = a_position;
	bse::Vec2 posB = b_position;

  Mat22 RotA = boxA->getRotationMatrix(),
           RotB = boxB->getRotationMatrix();

  Mat22 RotAT = RotA.transpose();
  Mat22 RotBT = RotB.transpose();

	bse::Vec2 dp = posB - posA;
	bse::Vec2 dA = RotAT.mul(dp);
	bse::Vec2 dB = RotBT.mul(dp);

	Mat22 C = RotAT.mul(RotB);
	Mat22 absC = bseAbs(C);
	Mat22 absCT = absC.transpose();

	// Box A faces
	bse::Vec2 faceA = bseAbs(dA) - hA - bseMul(absC, hB);
	if (faceA.x > 0.0f || faceA.y > 0.0f)
		return 0;

	// Box B faces
	bse::Vec2 faceB = bseAbs(dB) - bseMul(absCT, hA) - hB;
	if (faceB.x > 0.0f || faceB.y > 0.0f)
		return 0;

	// Find best axis
	Axis axis;
	float separation;
	bse::Vec2 normal;

	// Box A faces
	axis = FACE_A_X;
	separation = faceA.x;
	normal = dA.x > 0.0f ? RotA.col1 : -RotA.col1;

	const float relativeTol = 0.9999f;
	const float absoluteTol = 0;//0.0001f;

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	{
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = dA.y > 0.0f ? RotA.col2 : -RotA.col2;
	}

	// Box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	{
		axis = FACE_B_X;
		separation = faceB.x;
		normal = dB.x > 0.0f ? RotB.col1 : -RotB.col1;
	}

	if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	{
		axis = FACE_B_Y;
		separation = faceB.y;
		normal = dB.y > 0.0f ? RotB.col2 : -RotB.col2;
	}

	// Setup clipping plane data based on the separating axis
	bse::Vec2 frontNormal, sideNormal;
	ClipVertex incidentEdge[2];
	float front, negSide, posSide;
	char negEdge, posEdge;

	// Compute the clipping lines and the line segment to be clipped.
	switch (axis)
	{
	case FACE_A_X:
		{
			frontNormal = normal;
			front = bseDot(posA, frontNormal) + hA.x;
			sideNormal = RotA.col2;
			float side = bseDot(posA, sideNormal);
			negSide = -side + hA.y;
			posSide =  side + hA.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			bseComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;

	case FACE_A_Y:
		{
			frontNormal = normal;
			front = bseDot(posA, frontNormal) + hA.y;
			sideNormal = RotA.col1;
			float side = bseDot(posA, sideNormal);
			negSide = -side + hA.x;
			posSide =  side + hA.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			bseComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;

	case FACE_B_X:
		{
			frontNormal = -normal;
			front = bseDot(posB, frontNormal) + hB.x;
			sideNormal = RotB.col2;
			float side = bseDot(posB, sideNormal);
			negSide = -side + hB.y;
			posSide =  side + hB.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			bseComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;

	case FACE_B_Y:
		{
			frontNormal = -normal;
			front = bseDot(posB, frontNormal) + hB.y;
			sideNormal = RotB.col1;
			float side = bseDot(posB, sideNormal);
			negSide = -side + hB.x;
			posSide =  side + hB.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			bseComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;
	}

	// clip other face with 5 box planes (1 face plane, 4 edge planes)

	ClipVertex clipPoints1[2];
	ClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = bseClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	if (np < 2)
		return 0;

	// Clip to negative box side 1
	np = bseClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, posSide, posEdge);

	if (np < 2)
		return 0;

	// Now clipPoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	int numContacts = 0;
	for (int i = 0; i < 2; ++i)
	{
		float separation = bseDot(frontNormal, clipPoints2[i].v) - front;

		if (separation <= 0)
		{
      contacts[numContacts].body1 = boxA->getBody();
      contacts[numContacts].body2 = boxB->getBody();
      contacts[numContacts].penetration = -separation;
      contacts[numContacts].contactNormal = -normal;
			// slide contact point onto reference face (easy to cull)
      contacts[numContacts].contactPoint = clipPoints2[i].v - separation * frontNormal;
			contacts[numContacts].feature = clipPoints2[i].fp;
			if (axis == FACE_B_X || axis == FACE_B_Y)
				flipFeature(contacts[numContacts].feature);
			++numContacts;
		}
	}

	return numContacts;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderBoxBox(const Box* box1, const Box* box2, ContactsPool* pool, CollisionData* data)
{
  Contact contacts[2];
  int numContacts = bseCollideBoxBox(box1, box2, contacts);
  for (int i=0; i<numContacts; ++i)
  {
    data->contacts[i] = pool->getContactFromPool();
    *(data->contacts[i]) = contacts[i];
  }
  return numContacts;
}


//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderCirclePolygon(
  const Circle* circle, const Polygon* polygon,
  ContactsPool* pool, CollisionData* data)
{
  bse::Vec2  pCircle = circle->getPosition();
  bse::Real  radius = circle->getRadius();

  bse::Vec2  pPoly = polygon->getPosition();
  Mat22 oPoly = polygon->getRotationMatrix();

  // bring the circle in polygon world
  bse::Vec2 pos = bseMulT(oPoly, (pCircle-pPoly));

  // check all the polygon axis for separation, take the closest one
  int maxPenEdge = -1;
  bse::Real maxPenVal = 0;//FLT_MAX;
  bse::Vec2 maxPoint(0,0);

  const bseVertexesList vList = polygon->getVertexesList();
  int numVerts = (int)vList.size();
  bse::Real radiusSqr = radius * radius;

  // TODO: (optimization) store some constant values, like VjVi, VjVi.length(), ...
  for (int i=0; i<numVerts; ++i)
  {
    bse::Vec2 v1 = vList[i];
    bse::Vec2 v2 = vList[(i+1)%numVerts];

    bse::Vec2 v2v1 = v2 - v1;
    bse::Real v2v1_L = v2v1.mag();

    // project circle pos on the edge
    bse::Vec2 posToV1 = pos - v1;
    //    bse::Real posToV1Sqr = posToV1.sqrmag();
    bse::Real prj = bseDot(posToV1, v2v1) / (v2v1_L*v2v1_L);
    //    bse::Real prj = ((pos.x-v1.x)*(v2v1.x) + (pos.y-v1.y)*(v2v1.y))/post
    bse::Vec2 pointToCheck;
    if (prj<0.0f)
    {
      pointToCheck = v1;
    } else if (prj>1.0f)
    {
      pointToCheck = v2;
    } else
    {
      // min distance point lies on the edge,
      pointToCheck = v1 + v2v1 * prj;
    }

    bse::Real distSqr = (pointToCheck-pos).sqrmag();
    if (distSqr < radiusSqr)
    {
      bse::Real penVal = radius - sqrt(distSqr);
      if (penVal > maxPenVal)
      {
        maxPenEdge = i;
        maxPenVal = penVal;
        maxPoint = pointToCheck;
      }
    }
  }

  if (maxPenEdge != -1)
  {
    data->contacts[0] = pool->getContactFromPool();
    Contact* contact = data->contacts[0];
    contact->contactPoint = pPoly + bseMul( oPoly, maxPoint );
    contact->contactNormal = (pCircle - contact->contactPoint);
    contact->contactNormal.normalize();
    contact->penetration = maxPenVal;
    contact->body1 = circle->getBody();
    contact->body2 = polygon->getBody();
    return 1;
  }

  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderBoxPolygon(
  const Box* box, const Polygon* polygon,
  ContactsPool* pool, CollisionData* data)
{
  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderPolygonPointPolygon(
  const bse::Vec2& point, const Polygon* polygon,
  Contact* contact)
{
  // bring the point in polygon space
  bse::Vec2 pPoly = polygon->getPosition();
  Mat22 oPoly = polygon->getRotationMatrix();
  bse::Vec2 pos = bseMulT(oPoly, (point-pPoly));

  // I need the normal for each edge, as in 3D, normal should be defined by the vertexes order.
  // The polygon needs to be convex. TODO put a test of convexity in place, and assert if it fails.
  bse::Vec2 normal, edge, pRel;
  bse::Vec2 minNormal(0,1);
  bse::Real minPen = FLT_MAX;

  const bseVertexesList vList = polygon->getVertexesList();
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
      return 0; // point outside the polygon

    bse::Real pen = -dot;
    if (pen < minPen)
    {
      minPen = pen;
      minNormal = normal;
    }
  }

  // if I reach here, for sure all the edge-constraints are satisfied
  // so I can fill the contact
  contact->contactNormal = bseMul(oPoly, minNormal);
  contact->contactPoint = point; // it's already in world space
  contact->penetration = minPen;

  return 1;
}

//---------------------------------------------------------------------------------------------------------------------
int NarrowPhaseCollider::colliderPolygonPolygon(
  const Polygon* polygon1, const Polygon* polygon2,
  ContactsPool* pool, CollisionData* data)
{
  bse::Vec2 p1 = polygon1->getPosition();
  Mat22 o1 = polygon1->getRotationMatrix();
  bse::Vec2 p2 = polygon2->getPosition();
  Mat22 o2 = polygon2->getRotationMatrix();
  bse::UInt computedContacts = 0;

#ifdef USE_GJK
  // TODO: my gjk implementation
  return 0;
#else

  // in order to merge similar contacts
  Contact tempContact[2];
  bool       belongToFirst[2];

  // naive intersection, o(n^2) of course...
  // 1. test all the vertices of polygon one against all the edges of polygon two
  const bseVertexesList vList1 = polygon1->getVertexesList();
  const bse::UInt numVerts1 = (bse::UInt)vList1.size();
  for (bse::UInt vIndex1 = 0; vIndex1 < numVerts1; ++vIndex1)
  {
    // transform the point in world space (then the routine will transform it in it's local space)
    bse::Vec2 vert1 = p1 + bseMul(o1, vList1[vIndex1]);
    int numContacts = colliderPolygonPointPolygon(vert1, polygon2, &tempContact[computedContacts]);
    if (numContacts==1)
    {
      //data->contacts[computedContacts] = pool->getContactFromPool();
      //bseContact* contact = data->contacts[computedContacts];

      //contact->contactPoint = vert1; //vList1[vIndex1]; //tempContact.contactPoint; //pPoly + bseMul( oPoly, maxPoint );
      //contact->contactNormal = tempContact.contactNormal;
      //contact->penetration = tempContact.penetration;
      tempContact[computedContacts].body1 = polygon1->getBody();
      tempContact[computedContacts].body2 = polygon2->getBody();
//        tempContact[computedContacts].contactNormal = -tempContact[computedContacts].contactNormal;
      belongToFirst[computedContacts] = true;
      ++computedContacts;
//      return computedContacts; // for now, don't care of other contacts
    }
  }

  if (computedContacts<2)
  {
    // 2. test all the vertices of polygon two against all the edges of polygon one
    const bseVertexesList vList2 = polygon2->getVertexesList();
    const bse::UInt numVerts2 = (bse::UInt)vList2.size();
    for (bse::UInt vIndex2 = 0; vIndex2 < numVerts2; ++vIndex2)
    {
      // transform the point in world space (then the routine will transform it in it's local space)
      bse::Vec2 vert2 = p2 + bseMul(o2, vList2[vIndex2]);
      int numContacts = colliderPolygonPointPolygon(vert2, polygon1, &tempContact[computedContacts]);
      if (numContacts==1)
      {
        // if I'm here, no contact already allocated
        //data->contacts[computedContacts] = pool->getContactFromPool();
        //bseContact* contact = data->contacts[computedContacts];
        //contact->contactPoint = vert2;
        //contact->contactNormal = tempContact.contactNormal;
        //contact->penetration = tempContact.penetration;
          // inverse order...
        tempContact[computedContacts].body1 = polygon2->getBody();
        tempContact[computedContacts].body2 = polygon1->getBody();
        belongToFirst[computedContacts] = false;
        ++computedContacts;

        if (computedContacts==2)
          break;
    //    return computedContacts; // for now, don't care of other contacts
      }
    }
  }

  // ok, I can have 0,1,2 contacts
  // if 0 or 1, no prob, allocate the contact (if 1) and copy the data
  // if 2, then i need an extra check, if the contacts are very close to each other
  // i need to do some sort of merge, because it's a vertex vertex contact (no vertex-edge)
  // so the normal could be a bit messed
  if (computedContacts==0)
    return 0;
  else if (computedContacts==1)
  {
    data->contacts[0] = pool->getContactFromPool();
    Contact* contact = data->contacts[0];
    *contact = tempContact[0];
    return 1;
  } else // it's 2
  {
    // check if it's an edge-edge contact
    // eg: the two normals are almost parallel
    if (fabs(bseCross(tempContact[0].contactNormal, tempContact[1].contactNormal))<0.01f)
    {
      // merge
     bse::Vec2 c1 = tempContact[0].contactPoint;
      bse::Vec2 c2 = tempContact[1].contactPoint;
      Contact* contact;
      data->contacts[0] = pool->getContactFromPool();
      contact = data->contacts[0];
      *contact = tempContact[0];
      contact->contactPoint = (c1+c2)*0.5f;

      if (belongToFirst[0])
      {
        contact->body1 = polygon1->getBody();
        contact->body2 = polygon2->getBody();
      } else
      {
        contact->body1 = polygon2->getBody();
        contact->body2 = polygon1->getBody();
      }

        // the normal must point toward polygon1
      //contact->contactNormal = -c2c1/c2c1L;
      contact->penetration = 0.5f*(tempContact[0].penetration + tempContact[1].penetration); // ???

      return 1;
    }

    if (belongToFirst[0] == belongToFirst[1])
    {
      Contact* contact;
      data->contacts[0] = pool->getContactFromPool();
      contact = data->contacts[0];
      *contact = tempContact[0];

      data->contacts[1] = pool->getContactFromPool();
      contact = data->contacts[1];
      *contact = tempContact[1];
      return 2;
    } else
    {
      // contacts belong to different polygon, good candidate for v-v
        // threshold value to consider a pair of contacts as a v-v
      const bse::Real VV_THRESHOLD = 0.025f;
      bse::Vec2 c1 = tempContact[0].contactPoint;
      bse::Vec2 c2 = tempContact[1].contactPoint;
      bse::Vec2 c2c1 = c2 - c1;
      bse::Real c2c1L = c2c1.mag();
      if (c2c1L < VV_THRESHOLD)
      {
        // merge
        Contact* contact;
        data->contacts[0] = pool->getContactFromPool();
        contact = data->contacts[0];
        *contact = tempContact[0];
        contact->contactPoint = (c1+c2)*0.5f;
           // already checked: are in different stages, so I can decide the order
        contact->body1 = polygon1->getBody();
        contact->body2 = polygon2->getBody();
          // the normal must point toward polygon1
        contact->contactNormal = -c2c1/c2c1L;
        contact->penetration = c2c1L; // ???

        return 1;
      } else
      {
        // don't merge
        Contact* contact;
        data->contacts[0] = pool->getContactFromPool();
        contact = data->contacts[0];
        *contact = tempContact[0];

        data->contacts[1] = pool->getContactFromPool();
        contact = data->contacts[1];
        *contact = tempContact[1];
        return 2;
      }
    }
  }

#endif // USE_GJK
  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
bool NarrowPhaseCollider::rayCastShape(
  const Shape* shape, const Ray* ray, RayCastResult* result)
{
  switch (shape->getType())
  {
  case BSE_SHAPE_CIRCLE:
    return rayCastCircle(static_cast<const Circle*>(shape), ray, result);
  case BSE_SHAPE_BOX:
    return rayCastBox(static_cast<const Box*>(shape), ray, result);
  case BSE_SHAPE_POLYGON:
    return rayCastPolygon(static_cast<const Polygon*>(shape), ray, result);
  default:
    break;
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool NarrowPhaseCollider::rayCastBox(const Box* box, const Ray* ray, RayCastResult* result)
{
  Mat22 ori = box->getRotationMatrix();
  bse::Vec2 pos = box->getPosition();
  bse::Vec2 half = box->getHalfDims();

  bse::Real xLow  = - half.x;
  bse::Real xHigh =   half.x;

  bse::Real yLow  = - half.y;
  bse::Real yHigh =   half.y;

  // transform the ray in box space (is this non efficient?)
  bse::Vec2 start = bseMulT(ori, ray->start-pos);
  bse::Vec2 end   = bseMulT(ori, ray->end-pos);
  bse::Vec2 dir   = end - start;

  bse::Real tnear = -FLT_MAX;
  bse::Real tfar = FLT_MAX;
  bse::Vec2 normal(0,1);

  // X planes

  bse::Real low[2] = {xLow, yLow};
  bse::Real high[2] = {xHigh, yHigh};
  bse::Real d[2] = {dir.x, dir.y};
  bse::Real e[2] = {end.x, end.y};
  bse::Real s[2] = {start.x, start.y};
  bse::Vec2 norm[2] = { bse::Vec2(-1,0), bse::Vec2(0,-1) };

  for (int i=0; i<2; ++i)
  {
    if (d[i] == 0.0f)  // ray parallel to X planes
    {
      // if ray it's not between the planes, no intersection
      if (e[i]<low[i] || e[i]>high[i])
        return false;
    }

    bse::Real t1 = (low[i] - s[i])/d[i];
    bse::Real t2 = (high[i] - s[i])/d[i];
    //normal = norm[i];

    if (t1>t2) {
      bse::Real temp=t2;
      t2=t1;
      t1=temp;
      // invert the normal
      norm[i].x = -norm[i].x;
      norm[i].y = -norm[i].y;
    }

    if (t1>tnear)
    {
      tnear = t1;
      normal = norm[i];
    }
    if (t2<tfar) tfar = t2;
    if (tnear>tfar)
      return false;
    if (tfar<0.0f)
      return false;
  }

  if (tnear>1.0f)
    return false; // clamp

  // ok, there's intersection, compute the point/normal
  bse::Vec2 point(dir.x*tnear, dir.y*tnear);
  point += start;
  result->point = pos + bseMul(ori, point);
  result->normal = bseMul(ori, normal);
  result->shape = box;

  return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool NarrowPhaseCollider::rayCastCircle(const Circle* shape, const Ray* ray, RayCastResult* result)
{
  // there is an intersection if:
  // 1) the distance between the center of the circle and the line is less than the radius
  // 2) the "time" of impact is between 0 and 1.
  bse::Vec2 pos = shape->getPosition();
  bse::Real radius = shape->getRadius();

  // put the segment in circle coordinates (simplest computation)
  bse::Vec2 start = ray->start - pos;
  bse::Vec2 end = ray->end - pos;
  bse::Vec2 dir = end - start;

  bse::Real a = dir.x * dir.x + dir.y * dir.y;
  bse::Real b = 2*(start.x * dir.x + start.y * dir.y);
  bse::Real c = start.x*start.x + start.y * start.y - radius*radius;

  bse::Real delta = b*b - 4*a*c;

  if (delta < 0.0f) return false; // no intersection

  bse::Real deltaSq = sqrt(delta);
  bse::Real rec2A = 1.0f / (2*a);
  bse::Real t0 = rec2A * (-b - deltaSq);

  if (t0>=0.0f && t0<=1.0f)
  {
    result->point = ray->start + bse::Vec2(dir.x*t0, dir.y*t0);
    result->normal = result->point - pos;
    result->normal.normalize();
    result->shape = shape;
    return true;
  } else
  {
    // compute the other root
    bse::Real t1 = rec2A * (-b + deltaSq);
    if (t1>=0.0f && t1<=1.0f)
    {
      result->point = ray->start + bse::Vec2(dir.x*t1, dir.y*t1);
      result->normal = result->point - pos;
      result->normal.normalize();
      result->shape = shape;
      return true;
    }
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool NarrowPhaseCollider::rayCastPolygon(const Polygon* poly, const Ray* ray, RayCastResult* result)
{
  // TODO
  Mat22 ori = poly->getRotationMatrix();
  bse::Vec2 pos = poly->getPosition();

  // transform the ray in polygon space.
  bse::Vec2 start = bseMulT(ori, ray->start-pos);
  bse::Vec2 end   = bseMulT(ori, ray->end-pos);
  bse::Vec2 dir   = end - start;

  result->shape = poly;
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void BroadPhaseCollider::update(ShapesList* shapes)
{
  doUpdate(shapes);
}

//---------------------------------------------------------------------------------------------------------------------
void BroadPhaseCollider::clear()
{
  m_shapePairsPool.resetShapePairsPool();
  doClear();
}

//---------------------------------------------------------------------------------------------------------------------
SimpleBPCollider::~SimpleBPCollider()
{

}

//---------------------------------------------------------------------------------------------------------------------
void SimpleBPCollider::doUpdate(ShapesList* shapes)
{
  m_shapePairsPool.resetShapePairsPool();

  // very simple broadphase, the worst possible scenario: everybody against everybody
  const size_t numShapes = shapes->size();
  for (size_t sIndex = 0; sIndex < numShapes; ++sIndex)
  {
    Shape* shape = (*shapes)[sIndex];
    shape->updateAABB();
  }

  for (size_t shapeIndex1 = 0; shapeIndex1 < numShapes; ++shapeIndex1)
  {
    Shape* shape1 = (*shapes)[shapeIndex1];

    for (size_t shapeIndex2 = shapeIndex1+1; shapeIndex2 < numShapes; ++shapeIndex2)
    {
      Shape* shape2 = (*shapes)[shapeIndex2];

      if (shape1->isStatic() && shape2->isStatic())
      {
        continue;
      }

      // check if this pair must be ignored
      if (m_scene->getPairCollisionMode(shape1, shape2) == BSE_PAIR_DISABLECOLLISION)
      {
        continue;
      }

      // check the aabb
      if (shape1->getAABB().intersect(shape2->getAABB()))
      {
        ShapesPair* pair = m_shapePairsPool.getShapesPairFromPool();
        pair->shape1 = shape1;
        pair->shape2 = shape2;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void SimpleBPCollider::doClear()
{
  // Nothing to do here.
}

//---------------------------------------------------------------------------------------------------------------------
SweepAndPruneBPCollider::~SweepAndPruneBPCollider()
{

}

//---------------------------------------------------------------------------------------------------------------------
void SweepAndPruneBPCollider::addShape(Shape* newShape)
{
  return m_sap.addShape(newShape);
}

//---------------------------------------------------------------------------------------------------------------------
bool SweepAndPruneBPCollider::removeShape(const Shape* shape)
{
  return m_sap.removeShape(shape);
}

//---------------------------------------------------------------------------------------------------------------------
void SweepAndPruneBPCollider::doUpdate(ShapesList* shapes)
{
  m_sap.updateSAP(shapes, &m_shapePairsPool);
}

//---------------------------------------------------------------------------------------------------------------------
void SweepAndPruneBPCollider::doClear()
{
  m_sap.clear();
}

}
}
