#pragma once

// #define PROFILING 1

#include <limits>
#include <vector>

#include "astar_planner/pose.hpp"
#include "utils/instrumentation.hpp"

namespace octo
{
struct Cell
{
  long index{ 0 };
  Pose pose{ 0, 0 };
  bool occupied{ false };
  std::vector<int> neighbours;
  double g_score{ std::numeric_limits<double>::max() };
  double f_score{ std::numeric_limits<double>::max() };

  bool operator()(const Cell* lhs, const Cell* rhs);
};
}  // namespace octo