/* $Id: systemdef.h,v 1.2 2011-03-31 08:15:58 okamoto Exp $ */
#ifndef systemdef_h
#define systemdef_h


#if (defined IRWAS_STANDALONE)

#ifndef USE_OPENGL
#define USE_OPENGL
#endif

#ifndef USE_ODE
#define USE_ODE
#endif

#ifndef EXEC_SIMULATION
#define EXEC_SIMULATION
#endif

#ifndef USE_XERCES
#define USE_XERCES
#endif

#elif (defined IRWAS_TEST_CLIENT)

#ifndef USE_ODE
#define USE_ODE
#endif

#ifndef USE_OPENGL
#define USE_OPENGL
#endif

#ifndef IMPLEMENT_DRAWER
#define IMPLEMENT_DRAWER
#endif

#elif (defined IRWAS_SIMSERVER)

#ifndef USE_ODE
#define USE_ODE
#endif

#ifndef EXEC_SIMULATION
#define EXEC_SIMULATION
#endif

#ifndef USE_XERCES
#define USE_XERCES
#endif

#endif

#ifdef USE_ODE
#include <ode/ode.h>
#else


#ifdef dDOUBLE
typedef double dReal;
#else
typedef float dReal;
#endif

typedef dReal dMatrix3[4*3];
typedef dReal dQuaternion[4];

#endif // USE_ODE

#ifdef USE_OPENGL

typedef int GLContext;
typedef GLContext DrawContext;

#ifndef IMPLEMENT_DRAWER
#define IMPLEMENT_DRAWER

#endif

#endif

#ifndef WIN32
typedef int SOCKET;
#else
#include <windows.h>	// for SOCKET symbol (WinSock)
#endif

#define CAPTURE_DEVICE_STR "capture device"


#endif /* systemdef_h */

