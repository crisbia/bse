// QuadTree.cpp : Defines the entry point for the console application.
//

#include <conio.h>

#include "QuadTree.h"

class MyObj
{
public:
  MyObj(const Vec& v1, const Vec& v2) : m_bounds(v1, v2)
  {
  }
  
  Bounds getBounds() const { return m_bounds; }
protected:
  Bounds m_bounds;
};

void main()
{
  QuadTree<MyObj, 1> quadTree;
  quadTree.setBounds( Bounds(Vec(0,0), Vec(10,10)) );

  std::vector<MyObj*> objects;
  objects.push_back(new MyObj(Vec(1,1),Vec(2,2)));
  objects.push_back(new MyObj(Vec(3,3),Vec(4,4)));
  quadTree.buildTree(objects);


  _getch();
}

