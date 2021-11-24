#ifndef _BSE_COMMONS_H_INCLUDED
#define _BSE_COMMONS_H_INCLUDED

#ifdef _DEBUG
#include <assert.h>
#define BSE_ASSERT(x) assert(x)
#else
#define BSE_ASSERT(x) ;
#endif

#endif // _BSE_COMMONS_H_INCLUDED