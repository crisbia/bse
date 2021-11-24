#ifndef _SYSUTILS_H_INCLUDED
#define _SYSUTILS_H_INCLUDED

typedef int(*myProc)();

int mygetch();
int getNumLogicalProcessors();
unsigned int getCurrentTime();
//myProc getProcAddress(const char* procName);

#endif // _SYSUTILS_H_INCLUDED
