#pragma once

#define PROFILING 1

#include <vector>
#include <unordered_map>
#include <iostream>
#include <sstream>

#include "utils/instrumentation.hpp"
#include "astar_planner/cell.hpp"

using neighbour_motion = std::pair<int, int>;

namespace octo
{
const neighbour_motion top_left = std::make_pair(1, -1);
const neighbour_motion top = std::make_pair(1, 0);
const neighbour_motion top_right = std::make_pair(1, 1);
const neighbour_motion right = std::make_pair(0, 1);
const neighbour_motion bottom_right = std::make_pair(-1, 1);
const neighbour_motion bottom = std::make_pair(-1, 0);
const neighbour_motion bottom_left = std::make_pair(-1, -1);
const neighbour_motion left = std::make_pair(0, -1);

class Map
{
public:
  int width_;
  int height_;
  double resolution_;
  std::vector<Cell> data_;
  void find_neighbours(int i, int j, std::vector<std::pair<int, int>>& neighbours);
  void print_map();
};
}