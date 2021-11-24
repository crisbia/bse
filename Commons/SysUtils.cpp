#include "SysUtils.h"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
//#include <GL/glx.h>
#include <string.h>
#include <sys/time.h>

int mygetch()
{
  struct termios oldt,
                 newt;
  int            ch;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
  return ch;
}

int getNumLogicalProcessors()
{
  return sysconf( _SC_NPROCESSORS_ONLN );
}

unsigned int getCurrentTime()
{
  timeval tv;
  gettimeofday(&tv, 0 );
  return tv.tv_sec * 1000 + (unsigned int)(tv.tv_usec / 1000);
}

// myProc getProcAddress(const char* procName)
// {
//   return (myProc)glXGetProcAddress((const GLubyte*)procName);
// }

// #elif defined(WIN32)

// #include <windows.h>

// int getNumLogicalProcessors()
// {
//   SYSTEM_INFO sysinfo;
//   GetSystemInfo(&sysinfo);

//   return static_cast<int>(sysinfo.dwNumberOfProcessors);
// }

// unsigned int getCurrentTime()
// {
//   return (unsigned int)timeGetTime();
// }

// myProc getProcAddress(const char* procName)
// {
//   return (myProc)wglGetProcAddress(procName);
// }

// #else
// #error platform not supported
// #endif
