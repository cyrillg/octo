// #define PROFILING 1

#include <cmath>
#include <iostream>
#include <string>

#include "astar_planner/planner.hpp"
#include "astar_viewer_config.hpp"

#define WIDTH 2000
#define HEIGHT 2000

int main(int /*argc*/, char* argv[])
{
  PROFILING_START("astart");
  std::cout << *argv << " Version " << astar_viewer_VERSION_MAJOR << "." << astar_viewer_VERSION_MINOR << std::endl;
  octo::Map map;
  octo::generate_map(WIDTH, HEIGHT, 1, { 0, 0 }, map);
  octo::Cell start = map.data_.at(0);
  octo::Cell goal = map.data_.at(map.data_.size() - 1);
  octo::plan(map, start, goal);
  PROFILING_END();

  return 0;
}