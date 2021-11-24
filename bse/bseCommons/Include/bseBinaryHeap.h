#ifndef _BSE_BINARYHEAP_H_INCLUDED
#define _BSE_BINARYHEAP_H_INCLUDED

#include <vector>
#include <functional>

namespace bse
{

/**
 * \brief Generic bynary heap structure
 */
template<typename T, class Compare, class ElementSwap>
class BinaryHeap
{
public:
  typedef std::vector<T>  ElementsVec;

  /**
   * Constructor
   */
  BinaryHeap(Compare comp = Compare(), ElementSwap elSwap = ElementSwap()) : 
      betterThan(comp), elementSwap(elSwap)
  {
  }

  /**
   * Destructor 
   */
  virtual ~BinaryHeap()
  {
  }

  void clear()
  {
    m_elements.clear();
  }

  size_t size() const
  {
    return m_elements.size();
  }

  void reserve(size_t sz)
  {
    m_elements.reserve(sz);
  }

  /**
   * Checks if the heap is empty 
   */
  bool empty() const
  {
    return m_elements.empty(); 
  }

  /**
   * Inserts an element in the heap.
   * After the insertion the heap is still valid.
   */
  void insert(const T& elem)
  {
    // insert at the end
    m_elements.push_back(elem);

    // move it to the right place
    size_t tPos = m_elements.size()-1;
    if (tPos == 0)
    {
      return;
    }

    bool goOn = true;
    do 
    {
      size_t parentPos = (tPos-1)/2;

      T& newElem = m_elements.at(tPos);
      T& parentElem = m_elements.at(parentPos);

      if (betterThan(newElem, parentElem))
      {
        elementSwap(newElem, parentElem);

        // swap them
        T temp = newElem;
        newElem = parentElem;
        parentElem = temp;

        // continue checking...
        tPos = parentPos;
        if (tPos==0)
        {
          goOn = false;
        }
      }
      else
      {
        goOn = false;
      }
    } while (goOn); 
  }

  /**
   * Removes the best element in the heap.
   * By construction, the best element is the root of the heap.
   */
  T removeBest()
  {
    T best = getBest();

    // replace the root with the last one
    elementSwap(m_elements.at(0), m_elements.back());  
    m_elements.at(0) = m_elements.back();
    m_elements.pop_back();
    size_t numElements = m_elements.size();
    
    // now fix the heap sending down the head value to the right place.
    if (numElements>0)
    {
      size_t bestPos = 0;
      bool goOn = true;
      while (goOn)
      {
        T& currentElement = m_elements.at(bestPos);
        size_t child1pos = bestPos*2 + 1;
        size_t child2pos = child1pos + 1;

        if (child2pos >= numElements)
        {
          if (child1pos >= numElements)
          {
            // can't swap...
            goOn = false;
            continue;
          } else
          {
            // best child is child1 and it's the only one, then we finish for sure
            T& child1 = m_elements.at(child1pos);
            if (betterThan(child1, currentElement))
            {
              elementSwap(currentElement, child1);  

              T temp = currentElement;
              currentElement = child1;
              child1 = temp;
            }

            goOn = false;
            continue;
          }
        }

        // two good children, which is the best one?
        T& child1 = m_elements.at(child1pos);
        T& child2 = m_elements.at(child2pos);
        if (betterThan(child1, child2))
        {
          if (betterThan(child1, currentElement))
          {
            elementSwap(currentElement, child1);

            T temp = currentElement;
            currentElement = child1;
            child1 = temp;

            bestPos = child1pos;
          } 
          else
          {
            goOn = false;
            continue; // current > child1 > child2 => no need to swap!
          }
        } else
        { 
          if (betterThan(child2, currentElement))
          {
            elementSwap(currentElement, child2);

            T temp = currentElement;
            currentElement = child2;
            child2 = temp;
            bestPos = child2pos;
          } 
          else
          {
            goOn = false; // current > child2 > child1 => no need to swap!
            continue;
          }
        }
      }
    }
 
    return best;
  }

  /*
   * Returns the best element of the heap.
   * It doesn't remove the element.
   */
  T getBest() const
  {
    return m_elements.front();
  }

  /**
   * Returns all the elements of the heap.
   */
  void getAllElements(ElementsVec& elements) const
  {
    elements = m_elements;
  }

  /**
   * Returns a reference to the vector of elements.
   * 
   */
  const ElementsVec& getElements() const
  {
    return m_elements;
  }

  T& getElement(size_t iElem) const
  {
    return m_elements.at(iElem);
  }

  /**
   * Signals that the priority of the element referred by index has increased.
   * Restores the validity of the heap. This is a special and very fast case
   * because it only requires to push the element up in the heap until it
   * make the heap valid (it's better than its children).
   */
  void onPriorityIncreased(size_t iElem) 
  {
    // move it to the right place
    size_t tPos = iElem;
    if (tPos == 0)
      return;
   
    bool goOn = true;
    do 
    {
      size_t parentPos = (tPos-1)/2;

      T& newElem = m_elements.at(tPos);
      T& parentElem = m_elements.at(parentPos);

      if (betterThan(newElem, parentElem))
      {
        elementSwap(parentElem, newElem);
        
        // swap them
        T temp = newElem;
        newElem = parentElem;
        parentElem = temp;

        // continue checking...
        tPos = parentPos;
        if (tPos==0)
        {
          goOn = false;
          continue;
        }
      }
      else
      {
        goOn = false;
        continue;
      }
    } while (goOn); 
  }

private:

  ElementsVec m_elements;

  Compare       betterThan;
  ElementSwap   elementSwap;
};

}

#endif // _BSE_BINARYHEAP_H_INCLUDED
