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

#define PROFILING 1

#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <cmath>

#include "astar_planner/priority_queue.hpp"
#include "astar_planner/map.hpp"
#include "astar_planner/cell.hpp"
#include "utils/instrumentation.hpp"

namespace octo
{
void generate_map(int width, int height, double resolution, Pose origin, Map& map);
double distance(const Cell& a, const Cell& b);
int plan(Map& map, const Cell& start, const Cell& goal);  // Pose start, Pose goal);

};  // namespace octo
