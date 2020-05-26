#include <cmath>
#include <iostream>
#include <string>

#include "astar_planner/planner.hpp"

#include "astar_viewer_config.hpp"

int main(int argc, char* argv[])
{
  // report version
  std::cout << argv[0] << " Version " << astar_viewer_VERSION_MAJOR << "."
    << astar_viewer_VERSION_MINOR << std::endl;
  //std::cout << "Usage: " << argv[0] << " number" << std::endl;
  //return 1;

// convert input to double
//const double inputValue = std::stod(argv[1]);

// calculate square root
  octo::plan();
  /*std::cout << "The square root of " << inputValue << " is " << outputValue
    << std::endl;*/

  return 0;
}