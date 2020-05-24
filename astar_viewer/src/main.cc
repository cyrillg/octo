#include <cmath>
#include <iostream>
#include <string>

#include "planner.hpp"

#include "astar_viewer_config.hpp"

int main(int argc, char *argv[]) {
#ifdef USELESS_VAR
  std::cout << "DEFINED!!!!!" << std::endl;
#endif // USELESS_VAR

  std::cout << "What are we doing here..." << std::endl;
  if (argc < 2) {
    // report version
    std::cout << argv[0] << " Version " << astar_viewer_VERSION_MAJOR << "."
              << astar_viewer_VERSION_MINOR << std::endl;
    std::cout << "Usage: " << argv[0] << " number" << std::endl;
    return 1;
  }

  // convert input to double
  const double inputValue = std::stod(argv[1]);

  // calculate square root
  const double outputValue = octo::plan(inputValue);
  std::cout << "The square root of " << inputValue << " is " << outputValue
            << std::endl;

  return 0;
}