#ifndef _BSE_BITFIELD_H_INCLUDED
#define _BSE_BITFIELD_H_INCLUDED

#include <vector>
#include <functional>
#include <string.h>

namespace bse
{

// TODO Port to std::bitset
class BitField
{
public:
  BitField(bse::UInt numBits) :
    wordSize(sizeof(bse::UInt))
  {
    m_numWords = numBits / (8 * wordSize) + 1;
    m_data = new bse::UInt[m_numWords];
    clear();
  }

  ~BitField()
  {
    delete [] m_data;
  }

  void setBit(bse::UInt i)
  {
    size_t p = i / (8 * wordSize);
    size_t o = i % (8 * wordSize);
    bse::UInt& w = m_data[p];
    const size_t b = 1 << o;
    w |= b;
  }

  void resetBit(bse::UInt i)
  {
    size_t p = i / (8 * wordSize);
    size_t o = i % (8 * wordSize);
    bse::UInt& w = m_data[p];
    const size_t b = ~(1 << o);
    w &= b;
  }

  bool hasBit(bse::UInt i) const
  {
    size_t p = i / (8 * wordSize);
    size_t o = i % (8 * wordSize);
    bse::UInt& w = m_data[p];
    const size_t b = 1 << o;
    return (w & b) != 0;
  }

  void clear()
  {
    memset(m_data, 0, wordSize * m_numWords);
  }

  void setAll()
  {
    char allSet = (char)-1;
    memset(m_data, allSet, wordSize * m_numWords);
  }

  bool allSet() const
  {
    for (size_t i=0; i<m_numWords; ++i)
    {
      if (m_data[i] != (bse::UInt)-1)
      {
        // Found a word that is not completely set to 1.
        return false;
      }
    }

    return true;
  }

  bool allClear() const
  {
    for (size_t i=0; i<m_numWords; ++i)
    {
      if (m_data[i] != 0)
      {
        // Found a word that is not completely set to 0.
        return false;
      }
    }

    return true;
  }

private:
  bse::UInt* m_data;
  size_t m_numWords;
  const size_t wordSize;
};

}

#endif // _BSE_BITFIELD_H_INCLUDED
