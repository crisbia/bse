#ifndef _H_QUADTREE_INCLUDED
#define _H_QUADTREE_INCLUDED

#include <vector>

class Vec
{
public:
  Vec() : x(0), y(0)
  {
  
  }
  Vec(const Vec& c)
  {
    x = c.x;
    y = c.y;
  }
  
  Vec(const float a, const float b) : x(a), y(b)
  {
  }

  float x, y;
};



class Bounds
{
public:
  Bounds(const Bounds& c)
  {
    lower = c.lower;
    upper = c.upper;
  }

  Bounds(const Vec& l, const Vec& u) : lower(l), upper(u)
  {
  }

  Bounds() : lower(Vec(0,0)), upper(Vec(0,0))
  {
  }

  Vec getSize() { return Vec(upper.x-lower.x, upper.y-lower.y); }

  bool intersect(const Bounds& bounds)
  {
    if (lower.x>bounds.upper.x || upper.x < bounds.lower.x || lower.y>bounds.upper.y || upper.y < bounds.lower.y)
      return false;
    return true;
  }

  Vec lower;
  Vec upper;
};


//template <class T, const objLimit>
//typedef std::vector<TreeNode<T,objLimit>*> TreeNodeObjects;

const float MIN_SIZE = 0.01f;

template <class T, const int objLimit>
class TreeNode
{
public:
  TreeNode() : m_parentNode(0)
  {
  }
  
  TreeNode(const TreeNode<T,objLimit>* parent) : m_parentNode(parent)
  {
  }

  void getObjects(std::vector<const T*>& objects) const
  {
    objects = m_objectsList;
  }

  void getChildren(std::vector<const TreeNode<T,objLimit>* >& children) const
  {
    children = m_childrenList;
  }

/*
  void insertObject(const T* object)
  {
    // discard the object if it doesn't intersect
    Vec boundsSize = m_bounds.getSize();
    if (boundsSize.x > MIN_SIZE && boundsSize.y > MIN_SIZE)
    {
      Bounds objBounds = object->getBounds();
      if (!objBounds.intersect(m_bounds))
        return;
    }

    // non-leaf nodes don't contain objects
    if (m_childrenList.size()>0)
    {
      // check insertion in the children
      for (unsigned int childIndex=0; childIndex<m_childrenList.size(); ++childIndex)
        const_cast<TreeNode<T,objLimit>*>(m_childrenList[childIndex])->insertObject(object);
    
      return;
    }

    // this is still a leaf node
    m_objectsList.push_back(object);

    if (m_objectsList.size()<=objLimit)
      return;

    // create children and distribute objects in them
    buildChildren();
  }
*/

  void insertObject(const T* object)
  {
    // discard the object if it doesn't intersect
    Bounds objBounds = object->getBounds();
    if (!objBounds.intersect(m_bounds))
      return;



    Vec boundsSize = m_bounds.getSize();
    if (boundsSize.x > MIN_SIZE && boundsSize.y > MIN_SIZE)
    {
    }

    // non-leaf nodes don't contain objects
    if (m_childrenList.size()>0)
    {
      // check insertion in the children
      for (unsigned int childIndex=0; childIndex<m_childrenList.size(); ++childIndex)
        const_cast<TreeNode<T,objLimit>*>(m_childrenList[childIndex])->insertObject(object);
    
      return;
    }

    // this is still a leaf node
    m_objectsList.push_back(object);

    if (m_objectsList.size()<=objLimit)
      return;

    // create children and distribute objects in them
    buildChildren();
  }

  Bounds getBounds() const { return m_bounds; }
  void setBounds(const Bounds& bounds) { m_bounds = bounds; }
  void buildChildren()
  {
    Bounds parentBounds = getBounds();
    for (int i=0; i<4; ++i)
    {
      // TreeNode<T, objLimit>* 
      TreeNode<T, objLimit>*  child = new TreeNode<T, objLimit>(this);
      Bounds childBounds;
      switch (i)
      {
      case 0:
        {
          childBounds.lower = parentBounds.lower;
          childBounds.upper =
            Vec( 
              parentBounds.lower.x + 0.5f * (parentBounds.upper.x - parentBounds.lower.x),
              parentBounds.lower.y + 0.5f * (parentBounds.upper.y - parentBounds.lower.y)
            );
        }
        break;
      case 1:
        {
          childBounds.lower = 
            Vec( 
              parentBounds.lower.x + 0.5f * (parentBounds.upper.x - parentBounds.lower.x),
              parentBounds.lower.y
            );
            
         childBounds.upper =
            Vec( 
              parentBounds.upper.x,
              parentBounds.lower.y + 0.5f * (parentBounds.upper.y - parentBounds.lower.y)
            );
        }
        break;
      case 2:
        {
          childBounds.lower = 
            Vec( 
              parentBounds.lower.x + 0.5f * (parentBounds.upper.x - parentBounds.lower.x),
              parentBounds.lower.y + 0.5f * (parentBounds.upper.y - parentBounds.lower.y)
            );
            
          childBounds.upper = parentBounds.upper;
        }
       break;
      case 3:
        {
          childBounds.lower = 
            Vec( 
              parentBounds.lower.x,
              parentBounds.lower.y + 0.5f * (parentBounds.upper.y - parentBounds.lower.y)
            );
            
          childBounds.upper = 
            Vec( 
              parentBounds.lower.x + 0.5f * (parentBounds.upper.x - parentBounds.lower.x),
              parentBounds.upper.y
            );
        }
        break;
      }

      child->setBounds(childBounds);
      m_childrenList.push_back(child);
    }

    // move the objects across the children
    // ... actually... insert every object in every intersected children
    for (unsigned int objIndex = 0; objIndex < m_objectsList.size(); ++objIndex)
    {
      Bounds objBounds = m_objectsList[objIndex]->getBounds();
      for (unsigned int childIndex = 0; childIndex < m_childrenList.size(); ++childIndex)
      {
        const_cast<TreeNode<T,objLimit>*>(m_childrenList[childIndex])->insertObject(m_objectsList[objIndex]);
      }
    }

    m_objectsList.resize(0); // not needed anymore... objects stored in children nodes      
  }
protected:
  Bounds m_bounds;
  std::vector< const TreeNode<T,objLimit>* > m_childrenList;
  std::vector<const T*> m_objectsList;
  const TreeNode<T, objLimit>* m_parentNode;
};

template <class T, const int objLimit>
class QuadTree
{
public:
  QuadTree()
  {
    m_root = new TreeNode<T, objLimit>();
  }

  const TreeNode<T, objLimit>* getRoot() const { return m_root; }

  void buildTree(const std::vector<T*>& objects)
  {
    for (unsigned int objIndex=0; objIndex<objects.size(); ++objIndex)
      insertObject(objects[objIndex]);
  }

  void setBounds(const Bounds& bounds) { m_bounds = bounds; }
  void insertObject(const T* object)
  {
    Bounds objBounds = object->getBounds();
    m_root->setBounds(m_bounds);
    m_root->insertObject(object);
  }

protected:
  TreeNode<T, objLimit>* m_root;
  Bounds m_bounds;
};

#endif