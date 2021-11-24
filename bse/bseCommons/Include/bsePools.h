#ifndef _BSE_POOLS_H_INCLUDED
#define _BSE_POOLS_H_INCLUDED

#include <iterator>
#include <vector>

namespace bse
{
//---------------------------------------------------------------------------------------------------------------------
/**
 * \brief Pool suitable for objects of one frame life
 * objects created from this kind of pool can't be
 * actually released. the pool can be reset in one shot.
 */
template<class T, const int chunkSize>
class FrameObjectsPool
{
public:
  typedef std::vector<T*> PoolChunksList;

  FrameObjectsPool() :
    m_chunksSize(chunkSize),
    m_firstFreeSlot(0),
    m_numberOfFreeSlots(0),
    m_currentChunk(0)
  {
    addChunk();
  }

#if 0
  FrameObjectsPool(size_t initialNumChunks, size_t chunksSize) :
    m_chunksSize(chunksSize)
  {
    for (size_t iChunk=0; iChunk<initialNumChunks; ++iChunk)
      addChunk();
  }
#endif

  virtual ~FrameObjectsPool()
  {
    disposeAllChunks();
  }

  T* addChunk()
  {
    T* newChunk = new T[m_chunksSize];
    m_poolChunks.push_back(newChunk);
    m_numberOfFreeSlots = m_chunksSize;
    m_firstFreeSlot = newChunk;
    return newChunk;
  }

  T* getObject()
  {
    if (m_numberOfFreeSlots==0)
    {
      // add a new chunk only if we are at the end of the poolChunks
      if (m_currentChunk == m_poolChunks.size()-1)
      {
        addChunk();
        ++m_currentChunk;
      } 
      else
      {
        // reuse existing chunk
        ++m_currentChunk;
        m_firstFreeSlot = m_poolChunks[m_currentChunk];
        m_numberOfFreeSlots = m_chunksSize;
      }
    }

    T* obj = m_firstFreeSlot;
    m_firstFreeSlot ++;
    m_numberOfFreeSlots--;
    return obj;
  }

  void disposeAllChunks()
  {
    for (size_t i=0; i<m_poolChunks.size(); ++i)
    {
      T* chunk = m_poolChunks[i];
      delete [] chunk;
    }

    m_poolChunks.clear();

    m_firstFreeSlot = 0;
    m_numberOfFreeSlots = 0;
    m_currentChunk = 0;
  }

  // This is implemented to be as fast as possible
  // It doesn't deallocate memory, It just restores
  // the available chunks as free.
  void clear()
  {
    m_currentChunk = 0;
    m_firstFreeSlot = 0;

    // if there's at least a chunk already allocated
    // set it as usable from the beginning.
    if (m_poolChunks.size() > 0)
    {
      m_numberOfFreeSlots = m_chunksSize;
      m_firstFreeSlot = m_poolChunks[0];
    }
  }

private:
  FrameObjectsPool(const FrameObjectsPool& other) {}
  FrameObjectsPool& operator=( const FrameObjectsPool& ) {}

  PoolChunksList  m_poolChunks;
  const int       m_chunksSize;
  T*              m_firstFreeSlot;
  size_t          m_numberOfFreeSlots;
  size_t          m_currentChunk;
};

//---------------------------------------------------------------------------------------------------------------------
/**
 * \brief Pool suitable for persistent objects
 * like objs with a life of more than one frame
 * the objects can be created from this pool and also
 * released to be reused for other purposes.
 * TODO
 */
template<typename T>
class PersistentObjectsPool
{

};

}

#endif // _BSE_POOLS_H_INCLUDED
