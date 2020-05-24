#if defined(_WIN32)  // windows
#if defined(PLANNER_BUILD)
#define PLANNER_API __declspec(dllexport)
#else
#define PLANNER_API __declspec(dllimport)
#endif
#else  // non windows
#define PLANNER_API
#endif

#include <iostream>

namespace octo
{
double plan(double x);
}
