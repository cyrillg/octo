#include "astar_planner/planner.hpp"

// a hack square root calculation using simple operations
namespace octo
{
using ScoredIndex = std::pair<long long, double>;

void reconstruct_path(const Cell& goal, const Cell& start, std::unordered_map<long long, Cell> came_from)
{
  PROFILE_FUNCTION();
  Cell current = goal;
  // std::cout << "Waypoint: " << goal.pose_.x << " " << goal.pose_.y << std::endl;
  while (current.index_ != start.index_)
  {
    current = came_from.at(current.index_);
    // std::cout << "Waypoint: " << current.pose_.x << " " << current.pose_.y << std::endl;
  }
  std::cout << "Cost: " << goal.f_score_ << std::endl;
}

void generate_map(int width, int height, double resolution, Pose origin, Map& map)
{
  PROFILE_FUNCTION();
  map.width_ = width;
  map.height_ = height;
  {
    PROFILE_SCOPE("map_reserve");
    map.data_.reserve(static_cast<long long>(width) * height);
  }

  if (width < 2 || height < 2)
  {
    std::cout << "Map must at least be 2x2" << std::endl;
    exit(1);
  }

  {
    PROFILE_SCOPE("cell_create");
    double x;
    double y;
    for (int i = 0; i < height; i++)
    {
      y = origin.y;
      for (int j = 0; j < width; j++)
      {
        x = origin.x + j * resolution;
        y = origin.y + i * resolution;
        Cell cell = Cell(x, y, false, j + i * height);  // TODO measure time and allocations using that
                                                        // temporary variable vs directly feeding emplace_back
        map.data_.emplace_back(cell);
      }
    }
  }
  {
    PROFILE_SCOPE("find neighbours");
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        long long current_index = static_cast<long long>(j) + static_cast<long long>(i) * map.height_;

        std::vector<std::pair<int, int>> neighbours;
        map.find_neighbours(i, j, neighbours);

        for (auto neighbour : neighbours)
        {
          long long neighbour_index =
              j + static_cast<long long>(neighbour.second) + (static_cast<long long>(neighbour.first) + i) * height;
          map.data_[current_index].neighbours_.push_back(neighbour_index);
        }
      }
    }
  }
}

double distance(const Cell& a, const Cell& b)
{
  // std::cout << a.pose_.x << ", " << b.pose_.x << ", " << a.pose_.y << ", " << b.pose_.y << std::endl;
  return std::sqrt(std::pow(a.pose_.x - b.pose_.x, 2) + std::pow(a.pose_.y - b.pose_.y, 2));
}

// int plan(Pose start_pose, Pose goal_pose);
int plan(Map& map, const Cell& start, const Cell& goal)
{
  PROFILE_FUNCTION();

  auto compare = [](ScoredIndex a, ScoredIndex b)
  {
    return a.second > b.second;
  };
  map.data_[start.index_].g_score_ = 0.;
  map.data_[start.index_].f_score_ = distance(start, goal);

  std::priority_queue<ScoredIndex, std::vector<ScoredIndex>, decltype(compare)> open_set(compare);
  open_set.push(std::make_pair(start.index_, map.data_[start.index_].f_score_));

  std::unordered_map<long long, Cell> came_from;

  while (open_set.size())
  {
    ScoredIndex current_scored_index = open_set.top();
    Cell current = map.data_.at(current_scored_index.first);
    if (current.f_score_ != current_scored_index.second)
    {
      open_set.pop();
      continue;
    }

    // std::cout << current.index_ << " vs " << goal.index_ << std::endl;
    if (current.index_ == goal.index_)
    {
      std::cout << "Found!" << std::endl;
      reconstruct_path(map.data_.at(goal.index_), map.data_.at(start.index_), came_from);
      return 0;
    }

    open_set.pop();

    for (const long long neighbour_index : current.neighbours_)
    {
      Cell neighbour = map.data_.at(neighbour_index);
      double d = distance(current, neighbour);
      double new_score = current.g_score_ + d;
      if (new_score < neighbour.g_score_)
      {
        came_from[neighbour.index_] = current;
        map.data_[neighbour_index].g_score_ = new_score;
        double h = distance(neighbour, goal);
        map.data_[neighbour_index].f_score_ = new_score + h;
        open_set.push(std::make_pair(neighbour.index_, map.data_[neighbour_index].f_score_));
      }
    }
  }

  //  // Open set is empty but goal was never reached
  //  return failure
  std::cout << "Failed" << std::endl;
  return 1;
}
}  // namespace octo
