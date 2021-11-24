#ifndef _BSE_BUCKET_H_INCLUDED
#define _BSE_BUCKET_H_INCLUDED

#include <vector>
#include <functional>

namespace bse
{

// \brief Generic bucket of objects designed to speed up the research
template<typename T, class IsEqual, class ElementID>
class Bucket
{
public:
  typedef std::vector<T>  ElementsVec;

  Bucket(IsEqual equal = IsEqual(), ElementID elemID = ElementID()) :

      m_numberOfElements(0),isEqual(equal),elementID(elemID),
      m_minId(0), m_maxId(0),
      m_granularity(0)
  {
  }

  void initBuckets(bse::Int minId, bse::Int maxId, bse::UInt granularity)
  {
    m_minId = minId;
    m_maxId = maxId;
    m_granularity = granularity;

    m_buckets.resize(granularity);
  }

  virtual ~Bucket()
  {
  }

  // \brief Clear the buckets.
  void clear()
  {
    // Important: don't remove the buckets
    for (typename BucketsVec::iterator iter=m_buckets.begin(); iter != m_buckets.end(); ++iter)
    {
      (*iter).clear();
    }
  }

  // \brief Checks if the heap is empty
  bool empty() const
  {
    return (m_numberOfElements==0);
  }

  // \brief Inserts an element in the bucket. Search for the right bucket to push the element in.
  void insert(const T& elem)
  {
    ++m_numberOfElements;

    int value = elementID(elem);
    size_t iBuck = (value - m_minId) % m_granularity;
    m_buckets.at(iBuck).push_back(elem);
  }

  // \brief Removes an element from the bucket. This is the fast version, used when the containing bucket is known.
  bool removeFast(size_t iBucket, size_t iElem)
  {
    if (removeFromBuck(iBucket, iElem))
    {
      --m_numberOfElements;
      return true;
    }
    return false;
  }

  // \brief Removes an element from the bucket. This is the slow version, used when the containing bucket is not known.
  // It first does a research to find the bucket, then remove the element.
  bool remove(const T& elem)
  {
    T outElem;
    size_t outIBuck, outIElem;
    if (findElement(elem, outElem, outIBuck, outIElem))
    {
      removeFromBuck(outIBuck, outIElem);
      --m_numberOfElements;
      return true;
    }

    return false;
  }

  bool removeFromBuck(size_t iBuck, size_t iElem)
  {
    if (iBuck<m_buckets.size())
    {
      ElementsVec& buck = m_buckets.at(iBuck);
      size_t numElems = buck.size();
      if (iElem<numElems)
      {
        if (iElem<numElems-1)
        {
          // bring the end here
          buck.at(iElem) = buck.back();
        }

        // pop the end
        buck.pop_back();
        return true;
      }
    }

    return false;
  }

  /**
  * Finds an element.
  */
  bool findElement(const T& elem, T& outElem, size_t& outIBuck, size_t& outIElem) const
  {
    int value = elementID(elem);
    size_t iBuck = (value-m_minId) % m_granularity;
    const ElementsVec& buck = m_buckets.at(iBuck);
    size_t i=0;
    for (typename ElementsVec::const_iterator iter=buck.begin(); iter != buck.end(); ++iter)
    {
      if (isEqual(elem, (*iter)))
      {
        outElem = (*iter);
        outIBuck = iBuck; // out also the buck index, for fast removal
        outIElem = i;
        return true;
      }
      ++i;
    }

    // not found it if I'm here.
    return false;
  }

private:
  size_t                    m_numberOfElements;

  typedef typename std::vector<ElementsVec> BucketsVec;
  BucketsVec     m_buckets;
  const IsEqual  isEqual;
  ElementID      elementID;

  int m_minId;
  int m_maxId;
  unsigned int m_granularity;
};

}

#endif // _BSE_BUCKET_H_INCLUDED
