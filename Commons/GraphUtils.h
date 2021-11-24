#ifndef _GRAPHUTILS_H_INCLUDED
#define _GRAPHUTILS_H_INCLUDED

#if defined(WIN32)
#include <windows.h>
#endif
#include <GL/gl.h>

#include "bseCommons.h"
#include "bseMath.h"


class bseGraph;
class bseAIScene;

class GraphData
{
public:
  bseGraph* graph;

  // rendering helpers
  unsigned int vertCount;
  float*       verts;
  float*       cols;
};

GraphData createPathFinderGraph(
                              bseAIScene* scene,
                              int numRows,
                              int numCols,
                              bool bidirect,
                              bool horizontals,
                              bool verticals,
                              bool diagonals,
                              float probability
                              );


#endif // _GRAPHUTILS_H_INCLUDED
