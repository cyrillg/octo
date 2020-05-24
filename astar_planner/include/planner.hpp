#if defined(_WIN32)  // windows
#if defined(ASTAR_BUILD)
#define ASTAR_API __declspec(dllexport)
#else
#define ASTAR_API __declspec(dllimport)
#endif
#else  // non windows
#define DECLSPEC
#endif

#include <iostream>

namespace octo
{
double plan(double x);
}