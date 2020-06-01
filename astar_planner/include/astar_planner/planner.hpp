#pragma once

#if defined(_WIN32)  // windows
#if defined(PLANNER_BUILD)
#define PLANNER_API __declspec(dllexport)
#else
#define PLANNER_API __declspec(dllimport)
#endif
#else  // non windows
#define PLANNER_API
#endif

// #define PROFILING 1

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "astar_planner/cell.hpp"
#include "astar_planner/map.hpp"
#include "astar_planner/priority_queue.hpp"
#include "utils/instrumentation.hpp"

#define MAX_DIM_SIZE 30000

namespace octo
{
void generate_map(int width, int height, double resolution, Pose origin, Map& map);
double distance(const Cell& a, const Cell& b);
int plan(Map& map, const Cell& start, const Cell& goal);  // Pose start, Pose goal);

};  // namespace octo
