#pragma once

#define PROFILING 1

#include <limits>
#include <vector>

#include "utils/instrumentation.hpp"
#include "astar_planner/pose.hpp"

namespace octo
{
class Cell
{
public:
  long index_;
  Pose pose_;
  bool occupied_;
  double g_score_ = std::numeric_limits<double>::max();
  double f_score_ = std::numeric_limits<double>::max();
  std::vector<int> neighbours_;

  Cell();
  Cell(double x, double y, bool occupied, int index);
  bool operator()(const Cell* a, const Cell* b);
};

}  // namespace octo