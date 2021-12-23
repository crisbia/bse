#ifndef _DRAWUTILS_H_INCLUDED
#define _DRAWUTILS_H_INCLUDED

#include <OpenGL/gl.h>

#include <GLFW/glfw3.h>

#include "bseCommons.h"
#include "bseMath.h"

#include <string>

typedef enum
{
  SHAPE_DRAW_NONE            = 0,
  SHAPE_DRAW_WIREFRAME       = 1,
  SHAPE_DRAW_FILLED          = 2,
  SHAPE_DRAW_SEMITRANSPARENT = 4,
  SHAPE_DRAW_AABB            = 8,
  SHAPE_DRAW_NORMALS         = 16,
  SHAPE_DRAW_ALL             = 0xffffff
} shapeDrawFlags_enum;

typedef unsigned int shapeDrawFlags;

typedef enum
{
  BODY_DRAW_NONE       = 0,
  BODY_DRAW_FRAME      = 1,
  BODY_DRAW_VELOCITY   = 2,
  BODY_DRAW_SHAPES     = 4,
  BODY_DRAW_ALL        = 0xffffff
} bodyDrawFlags_enum;

typedef unsigned int bodyDrawFlags;

struct Color
{
	Color() { cx = 1.0f; cy = 1.0f; cz = 1.0f; }
	Color(bse::Real x, bse::Real y, bse::Real z) { cx = x; cy = y; cz = z; }
	bse::Real cx, cy, cz;
  Color darken(unsigned int factor)
  {
    float f = 1 - 0.1f * factor;
    return Color(cx*f, cy*f, cz*f);
  }
  Color lighten(unsigned int factor)
  {
    float f = 1.0f + 0.1f * factor;
    return Color(cx*f, cy*f, cz*f);
  }
};

#include <vector>
// TODO: this isn't efficient, but I don't want to use the internal version. I could probably move
// the definition from shape.h to some common place (visible to the AI framework as well)
typedef std::vector<bse::Vec2> VertexList;

void drawWorldAxis(const Color& c);
void drawFrame(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Real scale); // x = red, y = green
void drawString(int x, int y, const char *string, ...);
void drawString(int x, int y, const Color& col, const char *string, ...);


namespace bse
{
  class ProfilerTask;

  namespace phx
  {
    class Shape;
    class AABB;
    class Scene;
  }
}

// AI
void drawArrowCharacter(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Real scale, const Color& c); // draw x = red, y = green


  typedef enum {
    INVALIDE_RENDER_UPDATEMODE = -1,
    RENDER_UPDATEMODE_TIMED,
    RENDER_UPDATEMODE_CONTINUOS,
    RENDER_UPDATEMODE_MANUAL,
    NUM_RENDER_UPDATEMODES
  } RenderUpdateType;

typedef void (*KEYBOARDFUNCTYPE)(int key, int scancode, int action, int mods);
typedef void (*FRAMEFUNCTYPE)( void );
typedef void (*MOUSEFUNCTYPE)(GLFWwindow* window, int button, int action, int mods, double, double);
typedef void (*MOUSEMOTIONFUNCTYPE)(GLFWwindow*, double, double );
typedef void (*RESHAPEFUNCTYPE)( int, int );
typedef void (*SHUTDOWNFUNCTYPE) ();

class CameraDesc
{
public:
  CameraDesc() :
      fixed(false),
      zoomMin(0),
      zoomMax(0),
      zoomTick(0),
      startZoom(0),
      startX(0),
      startY(0),
      viewTick(0)
      {
        startZoom = 2;
        zoomTick = 0.5f;
        zoomMax = 100.0f;
        zoomMin = 0.5f;
        startX = 0.0f;
        startY = -1.0f;
        viewTick = 0.1f;
      }
  bool fixed;
  float zoomMin;
  float zoomMax;
  float zoomTick;
  float startZoom;
  float startX;
  float startY;
  float viewTick;
};

class RenderSceneDesc
{
public:
  RenderSceneDesc() :
      windowHeight(1024), windowWidth(768), windowStartX(0), windowStartY(0),
      updateMode(RENDER_UPDATEMODE_TIMED), updateTiming(1000 / 60),
      keyboardFunc(0), frameFunc(0), mouseFunc(0), mouseMotionFunc(0), reshapeFunc(0), shutdownFunc(0)
      {

      }

  std::string windowTitle;
  unsigned int windowHeight;
  unsigned int windowWidth;
  unsigned int windowStartX;
  unsigned int windowStartY;
  RenderUpdateType updateMode;
  unsigned int updateTiming;
  int argc;
  char** argv;

  KEYBOARDFUNCTYPE keyboardFunc;
  FRAMEFUNCTYPE frameFunc;
  MOUSEFUNCTYPE mouseFunc;
  MOUSEMOTIONFUNCTYPE mouseMotionFunc;
  RESHAPEFUNCTYPE reshapeFunc;
  SHUTDOWNFUNCTYPE shutdownFunc;

  CameraDesc camera;
};

class RenderScene
{
public:
  static RenderScene* createScene(const RenderSceneDesc* sceneDesc);
  void getSceneBounds(bse::Vec2& lower, bse::Vec2& higher);
  void setCameraPosition(float x, float y);
  void getRenderArea(float &left, float &right, float &bottom, float &top);

protected:
  RenderScene(const RenderSceneDesc* desc);

public:
  KEYBOARDFUNCTYPE customKeyboardFunc;
  FRAMEFUNCTYPE customFrameFunc;
  MOUSEFUNCTYPE customMouseFunc;
  MOUSEMOTIONFUNCTYPE customMouseMotionFunc;
  RESHAPEFUNCTYPE customReshapeFunc;
  SHUTDOWNFUNCTYPE customShutdownFunc;

  RenderSceneDesc sceneDesc;
  CameraDesc      cameraDesc;

  GLFWwindow* mainWindow;

  int width;
  int height;
  int tx, ty, tw, th;
  float viewX, viewY, viewZoom;
  std::string windowTitle;
};

void initDrawUtils(const RenderSceneDesc& desc);
void shutdownDrawUtils();

bse::Vec2 screenToWorld(int x, int y);

////////////////////////////////////
// QuadTree

#include "bseAABB.h"
#include "QuadTree.h"

class AABB
{
public:
  AABB(Vec l, Vec u) : m_bounds(l, u)
  {
  }

  Bounds getBounds() const
  {
    return m_bounds;
  }

  Bounds m_bounds;
};

typedef QuadTree<AABB, 2> AABBTree;
typedef TreeNode<AABB, 2> AABBTreeNode;

void drawQuadTree(const AABBTree* tree, const Color& treeColor, const Color& objectsColor);



bool isExtensionSupported(char* szTargetExtension);
//unsigned int buildVBO(unsigned int vertCount, unsigned int componentsPerVert, float* verts, float* colors);
void buildVBO(unsigned int vertCount, float* verts, float* vertCols, unsigned int &vboVerts, unsigned int &vboCols);
void renderVBO(unsigned int colVBO, unsigned int vertVBO, unsigned int vertCount);
void renderVAr(unsigned int vertCount, float* verts, float* cols);
unsigned int drawCircleIntoArray(const bse::Vec2& pos, const bse::Real& radius, unsigned int k_segs, const Color& c, float* verts, float* cols);



class TextManager
{
public:
  TextManager() : m_startX(5), m_startY(5), m_offset(10)
  {}

  void startDrawing(int startX, int startY, int offset);
  void drawText(const Color& col, const char *string, ...);
protected:
  int m_startX, m_startY, m_offset;
};

class DebugDraw
{
public:
  DebugDraw(unsigned int maxNumLines, unsigned int maxNumTriangles);
  ~DebugDraw();
  void clearAll();
  void drawLine(const bse::Vec2& start, const bse::Vec2& end, const Color& c=Color(1,1,1));
  void drawTriangle(const bse::Vec2& v1, const bse::Vec2& v2, const bse::Vec2& v3, const Color& c=Color(1,1,1));
  void drawCircle(const bse::Vec2& pos, const bse::Real& radius, unsigned int k_segs, const Color& c, bool fill=false);
  void drawPolygon(const bse::Vec2& pos, const bse::Mat22& ori, const VertexList& verts, const Color& c, bool fill=false);
  void drawBox(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Vec2& half, const Color& c, bool fill=false);
  void drawBox(const bse::Vec2& pos, const bse::Vec2& half, const Color& c, bool fill=false);
  void drawCross(const bse::Vec2& pos, float size, const Color& c);

  void renderAll();
  void renderTriangles();
  void renderLines();
  void clearLines();
  void clearTriangles();

protected:
  void growLinesBuffer(unsigned int growth);
  void growTrianglesBuffer(unsigned int growth);
  void resizeLinesBuffer(unsigned int newSize);
  void resizeTrianglesBuffer(unsigned int newSize);
  float* resizeBuffer(float* buff, unsigned currentSize, unsigned int newSize);


  unsigned int m_maxNumLines;
  unsigned int m_maxNumTriangles;
  float* m_linesVerts;
  float* m_linesCols;
  unsigned int m_currLine;
  float* m_triVerts;
  float* m_triCols;
  unsigned int m_currTri;
};

class PhysicsDebugDraw : public DebugDraw
{
public:
  PhysicsDebugDraw() : DebugDraw(100000, 10000)
  {

  }
  void drawShape(const bse::phx::Shape* shape, const Color& color, shapeDrawFlags drawFlags, float scale=1.0f);
  void drawBodies(bse::phx::Scene* scene, bodyDrawFlags bFlags=BODY_DRAW_ALL, shapeDrawFlags sFlags=SHAPE_DRAW_ALL);
  void drawShapes(bse::phx::Scene* scene, shapeDrawFlags sFlags=SHAPE_DRAW_ALL);
  void drawRays(bse::phx::Scene* scene);
  void drawAABB(const bse::phx::AABB& aabb, const Color& c);

};

int reportProfilerOutput(const bse::ProfilerTask* root, int startX, int startY, int offsetX, int offsetY, const Color& textCol);

#endif // _DRAWUTILS_H_INCLUDED
