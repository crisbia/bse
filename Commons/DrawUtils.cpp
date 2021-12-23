#include "DrawUtils.h"
#include "bsePhysics.h"

#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include "SysUtils.h"
#include <stdarg.h>
#include <stdio.h>

#include <iostream>

// render scene is a sort of singleton.
static RenderScene* gRenderScene = 0;

const bool bDrawNormals = false;

void handlerKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

void handlerFrame();
void handlerMouse(GLFWwindow*, int, int, int);
void handlerMouseMotion(GLFWwindow*, double, double);
void handlerReshape(GLFWwindow*, int, int);
void handlerTimer(int t);

//---------------------------------------------------------------------------------------------------------------------
void drawColorString(int x, int y, const Color& col, const char *buffer)
{
  int w = 0;
  int h = 0;
  glfwGetFramebufferSize(gRenderScene->mainWindow, &w, &h);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  gluOrtho2D(0, w, h, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glColor3f(col.cx, col.cy, col.cz);
  glRasterPos2i(x, y);
  bse::Int length = (bse::Int)strlen(buffer);
  // for (bse::Int i = 0; i < length; ++i)
  // {
  //   glutBitmapCharacter(GLUT_BITMAP_8_BY_13, buffer[i]);
  // }

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

//---------------------------------------------------------------------------------------------------------------------
void drawString(int x, int y, const Color& col, const char *string, ...)
{
  char buffer[128];

  va_list arg;
  va_start(arg, string);
#if _MSC_VER > 1000
  vsprintf_s(buffer, string, arg);
#else
  vsprintf(buffer, string, arg);
#endif

  va_end(arg);

  drawColorString(x,y, col, buffer);
}

//---------------------------------------------------------------------------------------------------------------------
void drawString(int x, int y, const char *string, ...)
{
  char buffer[128];

  va_list arg;
  va_start(arg, string);
#if _MSC_VER > 1000
  vsprintf_s(buffer, string, arg);
#else
  vsprintf(buffer, string, arg);
#endif

  va_end(arg);

  drawColorString(x,y, Color(0.9f, 0.6f, 0.6f), buffer);
}

//---------------------------------------------------------------------------------------------------------------------
void drawWorldAxis(const Color& c)
{
  glColor4f(c.cx, c.cy, c.cz, 1.0f);
  glBegin(GL_LINES);
  glVertex2f(-1, 0);
  glVertex2f(1, 0);
  glVertex2f(0, -1);
  glVertex2f(0, 1);
  glEnd();
}

//---------------------------------------------------------------------------------------------------------------------
void drawBoundingBox(const bse::Vec2& low, const bse::Vec2& high, const Color& c)
{
  bse::Vec2 v;
  glColor4f(c.cx, c.cy, c.cz, 1.0f);
  glBegin(GL_LINE_LOOP);
  v = bse::Vec2(low.x, low.y);
  glVertex2f(v.x, v.y);
  v = bse::Vec2(high.x, low.y);
  glVertex2f(v.x, v.y);
  v = bse::Vec2(high.x, high.y);
  glVertex2f(v.x, v.y);
  v = bse::Vec2(low.x, high.y);
  glVertex2f(v.x, v.y);
  glEnd();
}

//---------------------------------------------------------------------------------------------------------------------
unsigned int drawCircleIntoArray(const bse::Vec2& pos, const bse::Real& radius, unsigned int k_segs, const Color& c, float* verts, float* cols)
{
  bse::Vec2 x = pos;
  bse::Real r = radius;
	const bse::Real k_segments = float(k_segs); //16.0f;
	const bse::Real k_increment = 2.0f * bse_pi / k_segments;

  unsigned int vIncr = 0;
  unsigned int cIncr = 0;

  bse::Real theta = 0.0f;
	for (int i = 0; i < k_segments; ++i)
	{
		bse::Vec2 d1(r * cosf(theta), r * sinf(theta));
		bse::Vec2 d2(r * cosf(theta+k_increment), r * sinf(theta+k_increment));
		bse::Vec2 v1 = x + d1;
    bse::Vec2 v2 = x + d2;
    verts[vIncr]   = v1.x;
    verts[vIncr+1] = v1.y;
    verts[vIncr+2] = v2.x;
    verts[vIncr+3] = v2.y;

    cols[cIncr]   = c.cx;
    cols[cIncr+1] = c.cy;
    cols[cIncr+2] = c.cz;
    cols[cIncr+3]   = c.cx;
    cols[cIncr+4] = c.cy;
    cols[cIncr+5] = c.cz;

    vIncr += 4;
    cIncr += 6;
		theta += k_increment;
	}

  return 2*k_segs;
}

//---------------------------------------------------------------------------------------------------------------------
void drawLine(const bse::Vec2& pos, const bse::Vec2& posEnd, const Color& c, const bse::Real thick)
{
  if (thick!=0)
  {
    glPushAttrib(GL_LINE_BIT);
    glLineWidth(thick);
  }

  bse::Vec2 v;
  glColor4f(c.cx, c.cy, c.cz, 1.0f);
  glBegin(GL_LINES);
    glVertex2f(pos.x, pos.y);
    glVertex2f(posEnd.x, posEnd.y);
  glEnd();

  if (thick!=0)
  {
    glPopAttrib();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void drawFrame(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Real scale)
{

}

//---------------------------------------------------------------------------------------------------------------------
void drawArrowCharacter(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Real scale, const Color& c)
{
  const bse::Real height = 1.0f;
  const bse::Real width = 1.0f;

  VertexList verts;
  verts.push_back(bse::Vec2(0, height*scale));
  verts.push_back(bse::Vec2(-scale*width/2.0f, -scale*height/2.0f));
  verts.push_back(bse::Vec2(0.0f, 0.0f));
  verts.push_back(bse::Vec2(scale*width/2.0f, -scale*height/2.0f));

//  drawPolygon(pos, ori, verts, c);
}

//---------------------------------------------------------------------------------------------------------------------
RenderScene* RenderScene::createScene(const RenderSceneDesc* sceneDesc)
{
  return new RenderScene(sceneDesc);
}

//---------------------------------------------------------------------------------------------------------------------
RenderScene::RenderScene(const RenderSceneDesc* desc)
{
  sceneDesc = *desc;
  cameraDesc = sceneDesc.camera; // redundant but useful!
  windowTitle = sceneDesc.windowTitle;

  customKeyboardFunc = sceneDesc.keyboardFunc;
  customFrameFunc = sceneDesc.frameFunc;
  customMouseFunc = sceneDesc.mouseFunc;
  customMouseMotionFunc = sceneDesc.mouseMotionFunc;
  customReshapeFunc = sceneDesc.reshapeFunc;
  customShutdownFunc = sceneDesc.shutdownFunc;

  viewZoom = sceneDesc.camera.startZoom;
  viewX = sceneDesc.camera.startX;
  viewY = sceneDesc.camera.startY;
}

//---------------------------------------------------------------------------------------------------------------------
void handlerTimer(int t)
{
  //glutSetWindow(gRenderScene->mainWindow);
  //glutPostRedisplay();
  //glutTimerFunc(gRenderScene->sceneDesc.updateTiming, handlerTimer, 0);
}

//---------------------------------------------------------------------------------------------------------------------
GLFWwindow* initGraphicsBackend(const RenderSceneDesc& sceneDesc)
{
  //glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

  // Use a timer to control the frame rate.
  // if (sceneDesc.updateMode == RENDER_UPDATEMODE_TIMED)
  // {
  //   glutTimerFunc(sceneDesc.updateTiming, handlerTimer, 0);
  // }

  glfwInit();

  GLFWwindow* window = glfwCreateWindow(sceneDesc.windowWidth, sceneDesc.windowHeight, sceneDesc.windowTitle.c_str(), nullptr, nullptr);

  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, handlerReshape);
  glfwSetKeyCallback(window, handlerKeyboard);

  glfwSetMouseButtonCallback(window, handlerMouse);
  glfwSetCursorPosCallback(window, handlerMouseMotion);

  return window;
}

//---------------------------------------------------------------------------------------------------------------------
void RenderScene::setCameraPosition(float x, float y)
{
  viewX = x;
  viewY = y;
  handlerReshape(gRenderScene->mainWindow, width, height);
}

//---------------------------------------------------------------------------------------------------------------------
void RenderScene::getSceneBounds(bse::Vec2& lower, bse::Vec2& higher)
{
  double modelview[16], projection[16];
  int viewport[4];

	//get the projection matrix
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
	//get the modelview matrix
  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	//get the viewport
  glGetIntegerv( GL_VIEWPORT, viewport );

  int lx = viewport[0],
      ly = viewport[1],
      ux = viewport[0] + viewport[2],
      uy = viewport[1] + viewport[3];
  double lowX, lowY, lowZ, highX, highY, highZ;
  gluUnProject(lx, ly, 0.5, modelview, projection, viewport, &lowX, &lowY, &lowZ);
  gluUnProject(ux, uy, 0.5, modelview, projection, viewport, &highX, &highY, &highZ);

  lower = bse::Vec2((float)lowX,(float)lowY);
  higher = bse::Vec2((float)highX,(float)highY);
}

//---------------------------------------------------------------------------------------------------------------------
void handlerKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (gRenderScene->customKeyboardFunc)
  {
    gRenderScene->customKeyboardFunc(key, scancode, action, mods);
  }

  if (action == GLFW_PRESS)
  {
    if (!gRenderScene->cameraDesc.fixed)
    {
      switch (key)
      {
        case GLFW_KEY_A:
          gRenderScene->viewZoom =
            bseMax(gRenderScene->viewZoom - gRenderScene->cameraDesc.zoomTick, gRenderScene->cameraDesc.zoomMin);
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;

          // Press 'z' to zoom out.
        case GLFW_KEY_Z:
          gRenderScene->viewZoom =
            bseMin(gRenderScene->viewZoom + gRenderScene->cameraDesc.zoomTick, gRenderScene->cameraDesc.zoomMax);
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
        case GLFW_KEY_LEFT:
          gRenderScene->viewX += gRenderScene->cameraDesc.viewTick;
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
          // Press right to pan right.
        case GLFW_KEY_RIGHT:
          gRenderScene->viewX -= gRenderScene->cameraDesc.viewTick;
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
          // Press down to pan down.
        case GLFW_KEY_DOWN:
          gRenderScene->viewY += gRenderScene->cameraDesc.viewTick;
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
          // Press up to pan up.
        case GLFW_KEY_UP:
          gRenderScene->viewY -= gRenderScene->cameraDesc.viewTick;
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
          // Press home to reset the view.
        case GLFW_KEY_HOME:
          gRenderScene->viewZoom = gRenderScene->cameraDesc.startZoom;
          gRenderScene->viewX = gRenderScene->cameraDesc.startX;
          gRenderScene->viewY = gRenderScene->cameraDesc.startY;
          handlerReshape(gRenderScene->mainWindow, gRenderScene->width, gRenderScene->height);
          break;
      }
    }
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
      if (gRenderScene->customShutdownFunc)
      {
        gRenderScene->customShutdownFunc();
      }
      shutdownDrawUtils();
      exit(0);
      break;
    }

  }
}

//---------------------------------------------------------------------------------------------------------------------
void handlerFrame()
{
  // start scene render
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // custom rendering (client side)
  if (gRenderScene->customFrameFunc)
  {
    gRenderScene->customFrameFunc();
  }

  glfwSwapBuffers(gRenderScene->mainWindow);
}

//---------------------------------------------------------------------------------------------------------------------
void handlerMouse(GLFWwindow* window, int button, int action, int mods)
{
  if (gRenderScene->customMouseFunc)
  {
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    int ww,wh, fw, fh;
    glfwGetWindowSize(window, &ww, &wh);
    glfwGetFramebufferSize(window, &fw, &fh);
    double r = (double)fw / (double)ww;

    gRenderScene->customMouseFunc(window, button, action, mods, x * r, y * r);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void handlerMouseMotion(GLFWwindow* window, double x, double y)
{
  if (gRenderScene->customMouseMotionFunc)
  {
    int ww, wh, fw, fh;
    glfwGetWindowSize(window, &ww, &wh);
    glfwGetFramebufferSize(window, &fw, &fh);
    double r = (double)fw / (double)ww;

    gRenderScene->customMouseMotionFunc(window, x * r, y * r);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void handlerReshape(GLFWwindow* window, int w, int h)
{
  gRenderScene->width = w;
  gRenderScene->height = h;

  int curr_x = 0;
  int curr_y = 0;
  int curr_w = 0;
  int curr_h = 0;

  glfwGetFramebufferSize(gRenderScene->mainWindow, &curr_w, &curr_h);

  gRenderScene->tx = curr_x;
  gRenderScene->ty = curr_y;
  gRenderScene->tw = curr_w;
  gRenderScene->th = curr_h;

  glViewport( gRenderScene->tx, gRenderScene->ty, gRenderScene->tw, gRenderScene->th );

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  float left, right, bottom, top;
  gRenderScene->getRenderArea(left, right, bottom, top);

  gluOrtho2D(left, right, bottom, top);

  if (gRenderScene->customReshapeFunc)
  {
    gRenderScene->customReshapeFunc(w,h);
  }

  handlerFrame();
}

//---------------------------------------------------------------------------------------------------------------------
// Convert a window coordinate point into graphics world coordinates.
bse::Vec2 screenToWorld(int x, int y)
{
  bse::Vec2 p;
  bse::Real ratio = bse::Real(gRenderScene->tw) / bse::Real(gRenderScene->th);
  bse::Real u = x / bse::Real(gRenderScene->tw);
  bse::Real v = (gRenderScene->th - y) / bse::Real(gRenderScene->th);
  p.x = gRenderScene->viewZoom * (gRenderScene->viewX - ratio) * (1.0f - u) + gRenderScene->viewZoom * (ratio + gRenderScene->viewX) * u;
  p.y = gRenderScene->viewZoom * (gRenderScene->viewY - 0.1f) * (1.0f - v) + gRenderScene->viewZoom * (gRenderScene->viewY + 1.9f) * v;
  return p;
}

//---------------------------------------------------------------------------------------------------------------------
void RenderScene::getRenderArea(float &left, float &right, float &bottom, float &top)
{
  float ratio = (float)tw / (float)th;
  left    = viewZoom * (viewX - ratio);
  right   = viewZoom * (ratio + viewX);
  bottom  = viewZoom * (viewY - 0.1f);
  top     = viewZoom * (viewY + 1.9f);
}

//---------------------------------------------------------------------------------------------------------------------
void initDrawUtils(const RenderSceneDesc& desc)
{
  gRenderScene = RenderScene::createScene(&desc);

  gRenderScene->mainWindow = initGraphicsBackend(desc);

  glfwMakeContextCurrent(gRenderScene->mainWindow);

  int width, height;
  glfwGetFramebufferSize(gRenderScene->mainWindow, &width, &height);

  handlerReshape(gRenderScene->mainWindow, width, height);

  while (!glfwWindowShouldClose(gRenderScene->mainWindow))
  {
     glfwGetFramebufferSize(gRenderScene->mainWindow, &width, &height);
    float ratio = width / (float) height;

    handlerFrame();

    glfwPollEvents();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void shutdownDrawUtils()
{
  delete gRenderScene;
  gRenderScene = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void drawNode(const AABBTreeNode* node, const Color& treeColor, const Color& objectsColor)
{
  Bounds bounds = node->getBounds();
  bse::Vec2 low(bounds.lower.x, bounds.lower.y);
  bse::Vec2 high(bounds.upper.x, bounds.upper.y);
  drawBoundingBox(low, high, treeColor);

  std::vector<const AABBTreeNode*> children;
  node->getChildren(children);
  for (unsigned int childIndex=0; childIndex < children.size(); ++childIndex)
  {
    drawNode(children[childIndex], treeColor, objectsColor);
  }

  std::vector<const AABB*> objects;
  node->getObjects(objects);
  for (unsigned int objIndex=0; objIndex < objects.size(); ++objIndex)
  {
    Bounds bounds = objects[objIndex]->getBounds();
    bse::Vec2 low(bounds.lower.x, bounds.lower.y);
    bse::Vec2 high(bounds.upper.x, bounds.upper.y);
    drawBoundingBox(low, high, objectsColor);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void drawQuadTree(const AABBTree* tree, const Color& treeColor, const Color& objectsColor)
{
  drawNode(tree->getRoot(), treeColor, objectsColor);
}

void buildVBO(unsigned int vertCount, float* verts, float* vertCols, unsigned int &vboVerts, unsigned int &vboCols)
{
  // Generate And Bind The Vertex Buffer
	glGenBuffers( 1, &vboVerts );					// Get A Valid Name
	glBindBuffer( GL_ARRAY_BUFFER, vboVerts );			// Bind The Buffer
	glBufferData( GL_ARRAY_BUFFER, vertCount*2*sizeof(float), verts, GL_STATIC_DRAW );

  glGenBuffers(1, &vboCols);
  glBindBuffer( GL_ARRAY_BUFFER, vboCols );
  glBufferData( GL_ARRAY_BUFFER, vertCount*3*sizeof(float), vertCols, GL_STATIC_DRAW );
}

//---------------------------------------------------------------------------------------------------------------------
void renderVBO(unsigned int colVBO, unsigned int vertVBO, unsigned int vertCount)
{
  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_COLOR_ARRAY );

  glBindBuffer( GL_ARRAY_BUFFER, colVBO );
  glColorPointer( 3, GL_FLOAT, 0, 0);

  glBindBuffer( GL_ARRAY_BUFFER, vertVBO );
	glVertexPointer( 2, GL_FLOAT, 0, 0);		// Set The Vertex Pointer To The Vertex Buffer

  glDrawArrays( GL_LINES, 0, vertCount );

  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

//---------------------------------------------------------------------------------------------------------------------
void renderVAr(unsigned int vertCount, float* verts, float* cols)
{
  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_COLOR_ARRAY );

  glColorPointer( 3, GL_FLOAT, 0, cols);
  glVertexPointer( 2, GL_FLOAT, 0, verts);

  glDrawArrays( GL_LINES, 0, vertCount );

  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

  GLenum err = glGetError();
  if (err != GL_NO_ERROR)
  {
    printf("OpenGL error: %d\n", err);
  }
}

//---------------------------------------------------------------------------------------------------------------------
bool isExtensionSupported( char* szTargetExtension )
{
	const unsigned char *pszExtensions = NULL;
	const unsigned char *pszStart;
	unsigned char *pszWhere, *pszTerminator;

	// Extension names should not have spaces
	pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
	if( pszWhere || *szTargetExtension == '\0' )
		return false;

	// Get Extensions String
	pszExtensions = glGetString( GL_EXTENSIONS );

	// Search The Extensions String For An Exact Copy
	pszStart = pszExtensions;
	for(;;)
	{
		pszWhere = (unsigned char *) strstr( (const char *) pszStart, szTargetExtension );
		if( !pszWhere )
			break;
		pszTerminator = pszWhere + strlen( szTargetExtension );
		if( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
			if( *pszTerminator == ' ' || *pszTerminator == '\0' )
				return true;
		pszStart = pszTerminator;
	}
	return false;
}

//---------------------------------------------------------------------------------------------------------------------
void TextManager::startDrawing(int startX, int startY, int offset)
{
  m_startX = startX;
  m_startY = startY;
  m_offset = offset;
}

//---------------------------------------------------------------------------------------------------------------------
void TextManager::drawText(const Color& col, const char *string, ...)
{
  int x = m_startX;
  int y = m_startY;
  m_startY += m_offset;

	char buffer[128];

	va_list arg;
	va_start(arg, string);
#if _MSC_VER > 1000
	vsprintf_s(buffer, string, arg);
#else
  vsprintf(buffer, string, arg);
#endif

	va_end(arg);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int w = 0;
	int h = 0;
  glfwGetFramebufferSize(gRenderScene->mainWindow, &w, &h);

	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

  glColor3f(col.cx, col.cy, col.cz);
	glRasterPos2i(x, y);
	bse::Int length = (bse::Int)strlen(buffer);
	// for (bse::Int i = 0; i < length; ++i)
	// {
	// 	glutBitmapCharacter(GLUT_BITMAP_8_BY_13, buffer[i]);
	// }

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

//---------------------------------------------------------------------------------------------------------------------
DebugDraw::DebugDraw(unsigned int maxNumLines, unsigned int maxNumTriangles) :
  m_maxNumLines(maxNumLines), m_maxNumTriangles(maxNumTriangles)
{
  m_linesVerts = new float[4*m_maxNumLines];
  m_linesCols = new float[6*m_maxNumLines];
  m_currLine = 0;

  m_triVerts = new float[6*m_maxNumLines];
  m_triCols = new float[9*m_maxNumLines];
  m_currTri = 0;
}

//---------------------------------------------------------------------------------------------------------------------
DebugDraw::~DebugDraw()
{
  delete [] m_linesVerts;
  delete [] m_linesCols;
  delete [] m_triVerts;
  delete [] m_triCols;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::clearAll()
{
  clearLines();
  clearTriangles();
}

//---------------------------------------------------------------------------------------------------------------------
static void addVertex(const bse::Vec2& v, const Color& c, float* &verts, float* &cols)
{
  *verts = v.x;
  ++verts;
  *verts = v.y;
  ++verts;

  *cols = c.cx;
  ++cols;
  *cols = c.cy;
  ++cols;
  *cols = c.cz;
  ++cols;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawLine(const bse::Vec2& start, const bse::Vec2& end, const Color& c)
{
  if (m_currLine+1==m_maxNumLines)
    growLinesBuffer(1000);

  // ASSERT(m_currLine < m_maxNumLines);
  float* verts = &(m_linesVerts[m_currLine*4]);
  float* cols  = &(m_linesCols[m_currLine*6]);
  addVertex(start, c, verts, cols);
  addVertex(end, c, verts, cols);
  ++m_currLine;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawTriangle(const bse::Vec2& v1, const bse::Vec2& v2, const bse::Vec2& v3, const Color& c)
{
  if (m_currTri+1==m_maxNumTriangles)
    growTrianglesBuffer(1000);

  float* verts = &(m_triVerts[m_currTri*6]);
  float* cols  = &(m_triCols[m_currTri*9]);
  addVertex(v1, c, verts, cols);
  addVertex(v2, c, verts, cols);
  addVertex(v3, c, verts, cols);
  ++m_currTri;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawPolygon(const bse::Vec2& pos, const bse::Mat22& ori, const VertexList& verts, const Color& c, bool fill)
{
  bse::Vec2 v1,v2;
  if (fill)
  {
    // TODO
  }
  else
  {
    const int numVert = static_cast<int>(verts.size());
    for (int vertIndex=0; vertIndex<numVert-1; ++vertIndex)
    {
      v1 = verts[vertIndex];
      v1 = pos + ori.mul(v1);
      v2 = verts[vertIndex+1];
      v2 = pos + ori.mul(v2);
      drawLine(v1, v2, c);
    }

    v1 = verts[numVert-1];
    v1 = pos + bseMul(ori, v1);
    v2 = verts[0];
    v2 = pos + bseMul(ori, v2);
    drawLine(v1, v2, c);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawCircle(const bse::Vec2& pos, const bse::Real& radius, unsigned int k_segs, const Color& c, bool fill)
{
  bse::Vec2 x = pos;
  bse::Real r = radius;
	const bse::Real k_segments = 12.0f;
	const bse::Real k_increment = 2.0f * bse_pi / k_segments;

  bse::Real theta = 0.0f;
  if (fill)
  {
    bse::Vec2 old = x + bse::Vec2(r, 0);
    theta += k_increment;
    for (int i = 0; i < k_segments; ++i)
    {
      bse::Vec2 d(r * cosf(theta), r * sinf(theta));
      bse::Vec2 v = x + d;
      drawTriangle(old, v, x, c);
      old = v;
      theta += k_increment;
    }
  }
  else
  {
    bse::Vec2 old = x + bse::Vec2(r, 0);
    theta += k_increment;
	  for (int i = 0; i < k_segments; ++i)
	  {
		  bse::Vec2 d(r * cosf(theta), r * sinf(theta));
		  bse::Vec2 v = x + d;
      drawLine(old, v, c);
		  old = v;
      theta += k_increment;
	  }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawBox(const bse::Vec2& pos, const bse::Vec2& half, const Color& c, bool fill)
{
  bse::Mat22 ori;
  drawBox(pos, ori, half, c, fill);
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawCross(const bse::Vec2& pos, float size, const Color& c)
{
  float halfSize = size * 0.5f;
  drawLine(pos - bse::Vec2::YAxis * halfSize, pos + bse::Vec2::YAxis * halfSize);
  drawLine(pos - bse::Vec2::XAxis * halfSize, pos + bse::Vec2::XAxis * halfSize);
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::drawBox(const bse::Vec2& pos, const bse::Mat22& ori, const bse::Vec2& half, const Color& c, bool fill)
{
  bse::Vec2 v1,v2,v3,v4;
  v1 = bse::Vec2(half.x, half.y);
  v1 = pos + bseMul(ori, v1);
  v2 = bse::Vec2(-half.x, half.y);
  v2 = pos + bseMul(ori, v2);
  v3 = bse::Vec2(-half.x, -half.y);
  v3 = pos + bseMul(ori, v3);
  v4 = bse::Vec2(half.x, -half.y);
  v4 = pos + bseMul(ori, v4);

  if (fill)
  {
    drawTriangle(v1, v2, v3, c);
    drawTriangle(v3, v4, v1, c);
  }
  else
  {
    drawLine(v1,v2,c);
    drawLine(v2,v3,c);
    drawLine(v3,v4,c);
    drawLine(v4,v1,c);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::renderAll()
{
  // Draw triangles first, so overlapping lines will appear in front.
  renderTriangles();
  renderLines();
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::renderTriangles()
{
  unsigned int vertCount = m_currTri * 3;

//  glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_COLOR_ARRAY );

  glColorPointer( 3, GL_FLOAT, 0, m_triCols);
  glVertexPointer( 2, GL_FLOAT, 0, m_triVerts);

  glDrawArrays( GL_TRIANGLES, 0, vertCount );

  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

  GLenum err = glGetError();
  if (err != GL_NO_ERROR)
    printf("error drawing lines vertex array\n");
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::renderLines()
{
  unsigned int vertCount = m_currLine * 2;
  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_COLOR_ARRAY );

  glColorPointer( 3, GL_FLOAT, 0, m_linesCols);
  glVertexPointer( 2, GL_FLOAT, 0, m_linesVerts);

  glDrawArrays( GL_LINES, 0, vertCount );

  glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

  GLenum err = glGetError();
  if (err != GL_NO_ERROR)
    printf("error drawing lines vertex array\n");
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::clearLines()
{
  m_currLine = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::clearTriangles()
{
  m_currTri = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::growLinesBuffer(unsigned int growth)
{
  resizeLinesBuffer(m_maxNumLines + growth);
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::growTrianglesBuffer(unsigned int growth)
{
  resizeTrianglesBuffer(m_maxNumTriangles + growth);
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::resizeLinesBuffer(unsigned int newSize)
{
  if (newSize > m_maxNumLines)
  {
    // Realloc the buffer and copy the data to the newly allocated memory.
    m_linesVerts = resizeBuffer(m_linesVerts, 4*m_maxNumLines, 4*newSize);
    m_linesCols = resizeBuffer(m_linesCols, 6*m_maxNumLines, 6*newSize);

    // Set the new limit.
    m_maxNumLines = newSize;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void DebugDraw::resizeTrianglesBuffer(unsigned int newSize)
{
  if (newSize > m_maxNumTriangles)
  {
    // Realloc the buffer and copy the data to the newly allocated memory.
    m_triVerts = resizeBuffer(m_triVerts, 6*m_maxNumTriangles, 6*newSize);
    m_triCols = resizeBuffer(m_triCols, 9*m_maxNumTriangles, 9*newSize);

    // Set the new limit.
    m_maxNumTriangles = newSize;
  }
}

//---------------------------------------------------------------------------------------------------------------------
float* DebugDraw::resizeBuffer(float* buff, unsigned currentSize, unsigned int newSize)
{
  float* newBuff = new float[newSize];
  memcpy(newBuff, buff, sizeof(float)*currentSize);
  delete [] buff;
  return newBuff;
}

//---------------------------------------------------------------------------------------------------------------------
void PhysicsDebugDraw::drawShape(const bse::phx::Shape *shape, const Color &c, shapeDrawFlags drawFlags, float scale)
{
  bool fill = (drawFlags & SHAPE_DRAW_FILLED) != 0;
  bool wire = (drawFlags & SHAPE_DRAW_WIREFRAME) != 0;

  Color w = c;
  if (wire && fill)
  {
    w.cx = c.cx*0.5f;
    w.cy = c.cy*0.5f;
    w.cz = c.cz*0.5f;
  }

  switch (shape->getType())
  {
  case bse::phx::BSE_SHAPE_CIRCLE:
    {
      const bse::phx::Circle* circle = (const bse::phx::Circle*)shape;
      bse::Mat22 ori = circle->getRotationMatrix();
      bse::Vec2 x = circle->getPosition();
      bse::Real r = circle->getRadius();

      r *= scale;

      if (fill)
      {
        drawCircle(x, r, 12, c, true);
      }

      if (wire)
      {
        drawCircle(x, r, 12, w, false);
      }
    }
    break;
  case bse::phx::BSE_SHAPE_POLYGON:
    {
      const bse::phx::Polygon* poly = (const bse::phx::Polygon*)shape;
      const bse::Mat22 ori = poly->getRotationMatrix();
      const bse::Vec2 pos = poly->getPosition();
      const bse::phx::bseVertexesList vList = poly->getVertexesList();
      VertexList verts;
      for (int i=0; i<(int)vList.size(); ++i)
      {
        verts.push_back(vList[i] * scale);
      }

      drawPolygon(pos, ori, verts, c);
    }
    break;
  case bse::phx::BSE_SHAPE_BOX:
    {
      const bse::phx::Box* box = (const bse::phx::Box*)shape;
      const bse::Vec2 half = box->getHalfDims() * scale;
      const bse::Mat22 ori = box->getRotationMatrix();
      const bse::Vec2 pos = box->getPosition();

      if (fill)
      {
        drawBox(pos, ori, half, c, true);
      }

      if (wire)
      {
        drawBox(pos, ori, half, w, false);
      }
    }
    break;
  default:
    break;
  }
}

//---------------------------------------------------------------------------------------------------------------------
void PhysicsDebugDraw::drawBodies(bse::phx::Scene* scene, bodyDrawFlags bFlags, shapeDrawFlags sFlags)
{
  Color colWhite(1,1,1);
  Color colGreen(0,1,0);

  if (!scene) return;

  bse::phx::BodiesList &bodies = scene->getBodies();
  for(bse::phx::BodiesList::iterator bodiesIter=bodies.begin(); bodiesIter!=bodies.end(); ++bodiesIter)
  {
    bse::phx::Body* body = (*bodiesIter);
    if (bFlags & BODY_DRAW_SHAPES)
    {
      bse::phx::ShapesList &shapes = body->getShapes();
      for(bse::phx::ShapesList::iterator shapesIter=shapes.begin(); shapesIter != shapes.end(); ++shapesIter)
      {
        bse::phx::Shape *shape = (*shapesIter);
        if (sFlags & SHAPE_DRAW_AABB)
        {
          drawAABB(shape->getAABB(), colGreen);
        }
        drawShape(shape, Color(0.9f, 0.4f, 0.4f), sFlags);
      }
    }

    if (bFlags & BODY_DRAW_VELOCITY)
    {
      bse::Vec2 pos = body->getPosition();
      bse::Vec2 vel = body->getLinearVelocity();
      bse::Real ang = body->getAngularVelocity();
      // draw a line proportional to linear velocity
      drawLine(pos, pos+vel, Color(0,0,1));
      // draw a circle proportional to angular velocity
      const bse::Real SCALE = 0.01f;
      drawCircle(pos, fabs(ang)*SCALE, 6, Color(1,0,0));
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
void PhysicsDebugDraw::drawShapes(bse::phx::Scene* scene, shapeDrawFlags sFlags)
{
  Color colWhite(1,1,1);
  Color colGreen(0,1,0);

  if (!scene) return;

  bse::phx::ShapesList &shapes = scene->getShapes();
  for(bse::phx::ShapesList::iterator shapesIter=shapes.begin(); shapesIter != shapes.end(); ++shapesIter)
  {
    bse::phx::Shape* shape = (*shapesIter);
    if (sFlags & SHAPE_DRAW_AABB)
    {
      drawAABB(shape->getAABB(), colGreen);
    }
    drawShape(shape, colWhite, sFlags);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void PhysicsDebugDraw::drawRays(bse::phx::Scene* scene)
{
  Color red(1,0,0);

  bse::phx::RaysList &raysList = scene->getRays();
  for (bse::phx::RaysList::const_iterator iter = raysList.begin(); iter != raysList.end(); ++iter)
  {
    const bse::phx::Ray* ray = (*iter);
    drawLine(ray->start, ray->end, red);
    drawCircle(ray->start, 0.05f, 12, red);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void PhysicsDebugDraw::drawAABB(const bse::phx::AABB& aabb, const Color& c)
{
  bse::Vec2 v1(aabb.low.x, aabb.low.y);
  bse::Vec2 v2(aabb.high.x, aabb.low.y);
  bse::Vec2 v3(aabb.high.x, aabb.high.y);
  bse::Vec2 v4(aabb.low.x, aabb.high.y);

  drawLine(v1,v2,c);
  drawLine(v2,v3,c);
  drawLine(v3,v4,c);
  drawLine(v4,v1,c);
}

//---------------------------------------------------------------------------------------------------------------------
int reportProfilerOutput(const bse::ProfilerTask* root, int startX, int startY, int offsetX, int offsetY, const Color& textCol)
{
  if (root)
  {
    std::vector<bse::ProfilerTask*> subTasks;
    root->getSubTasks(subTasks);

    float rootCost = root->m_totalTime;
    if (root->getParent() == 0)
    {
      float totalTime = 0.0f;
      for (size_t taskIndex = 0; taskIndex < subTasks.size(); ++taskIndex)
      {
        const bse::ProfilerTask* subTask = subTasks[taskIndex];
        totalTime += subTask->m_totalTime;
      }
      rootCost = totalTime;
    }

    for (size_t taskIndex = 0; taskIndex < subTasks.size(); ++taskIndex)
    {
      const bse::ProfilerTask* subTask = subTasks[taskIndex];
      if (subTask->getName() && subTask->m_numberOfCalls>0)
      {
        drawString(startX, startY, textCol, "%s - %.5f - %d - %.2f %%", subTask->getName(), subTask->m_totalTime, subTask->m_numberOfCalls, subTask->m_totalTime / rootCost * 100.0f);
        startY += offsetY;
      }

      startY = reportProfilerOutput(subTask, startX+offsetX, startY, offsetX, offsetY, textCol);
    }
  }
  return startY;
}
