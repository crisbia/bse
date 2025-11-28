#ifndef _BSE_MEMORYMANAGER_H_INCLUDED
#define _BSE_MEMORYMANAGER_H_INCLUDED

#include <cstdlib>
#include <vector>

namespace bse
{
// abstract class for memory allocations
// this can be overridden by the user, but a
// a default implementation could be used, too.
class Allocator
{
public:
  virtual void* mallocDEBUG(size_t size, const char* fileName, int line) = 0;
  virtual void* malloc(size_t size) = 0;
  virtual void* realloc(void* memory, size_t size) = 0;
  virtual void free(void* memory) = 0;

  virtual ~Allocator()
  {
  }

};

class StandardAllocator : public Allocator
{
public:
 	virtual void* mallocDEBUG(size_t size, const char* fileName, int line)
  {
    // TODO: malloc debug version
    (void)line;
    (void)fileName;
    (void)size;
    return ::malloc(size);
  }

	virtual void* malloc(size_t size)
  {
    return ::malloc(size);
  }

  virtual void* realloc(void* memory, size_t size)
  {
    return ::realloc(memory, size);
  }

  virtual void free(void* memory)
  {
    ::free(memory);
  }

  virtual ~StandardAllocator()
  {
  }

};

}
#endif // _BSE_MEMORYMANAGER_H_INCLUDED
