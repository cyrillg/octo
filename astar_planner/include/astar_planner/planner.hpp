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
#include <array>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <memory>
#include <math.h>

namespace octo
{
struct Pose
{
  double x_;
  double y_;

  // public:
  //  Pose() = default;
  //  Pose(double x, double y);
};

class Cell
{
public:
  long index_;
  Pose pose_;
  bool occupied_;
  double g_score_ = std::numeric_limits<double>::max();
  double f_score_ = std::numeric_limits<double>::max();
  std::vector<std::shared_ptr<Cell>> neighbours_;

  Cell(double x, double y, bool occupied, int index);
  bool operator()(const Cell* a, const Cell* b);
};

using Map = std::vector<std::shared_ptr<Cell>>;

Map generate_map(int width, int height, double resolution, Pose origin);

double distance(const Cell& a, const Cell& b);

int plan();  // Pose start, Pose goal);
};           // namespace octo
