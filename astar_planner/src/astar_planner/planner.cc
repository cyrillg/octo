#include "astar_planner/planner.hpp"

#include "utils/instrumentation.hpp"

// a hack square root calculation using simple operations
namespace octo
{
using ScoredIndex = std::pair<long long, double>;

void reconstruct_path(const Cell& goal, const Cell& start, std::unordered_map<long long, Cell> came_from)
{
  Cell current = goal;
  // std::cout << "Waypoint: " << goal.pose.x << " " << goal.pose.y << std::endl;
  while (current.index != start.index)
  {
    current = came_from.at(current.index);
    // std::cout << "Waypoint: " << current.pose.x << " " << current.pose.y << std::endl;
  }
  std::cout << "Cost: " << goal.f_score << std::endl;
}

void generate_map(int width, int height, double resolution, Pose origin, Map& map)
{
  PROFILE_FUNCTION();
  map.width_ = width;
  map.height_ = height;
  {
    PROFILE_SCOPE("map_reserve");
    map.data_.reserve(width * height);
  }

  if (width < 2 || height < 2)
  {
    std::cout << "Map must at least be 2x2" << std::endl;
    exit(1);
  }

  if (width > MAX_DIM_SIZE || height > MAX_DIM_SIZE)
  {
    std::cout << "Map width and length can be at most " << MAX_DIM_SIZE << " pixels" << std::endl;
    exit(1);
  }

  {
    PROFILE_SCOPE("cell_create");
    double x = 0.;
    double y = 0.;
    std::vector<std::pair<int, int>> neighbour_motions;
    std::vector<int> neighbours;
    int current_index = 0;
    int neighbour_index = 0;
    for (int i = 0; i < height; i++)
    {
      y = origin.y + i * resolution;
      for (int j = 0; j < width; j++)
      {
        {
          PROFILE_SCOPE("calc coordinates");
          x = origin.x + j * resolution;
          current_index = j + i * map.height_;
        }

        {
          PROFILE_SCOPE("find neighbours");
          neighbour_motions = map.find_neighbours(i, j);
        }

        {
          PROFILE_SCOPE("transform neighbours");
          neighbours.reserve(neighbour_motions.size());
          for (auto neighbour : neighbour_motions)
          {
            neighbours.emplace_back(j + neighbour.second + (neighbour.first + i) * height);
          }
        }
        {
          PROFILE_SCOPE("push neighbours");
          map.data_.push_back({ current_index, x, y, false, neighbours });
        }

        {
          PROFILE_SCOPE("clear tmps");
          neighbour_motions.clear();
          neighbours.clear();
        }
      }
    }
  }
}

double distance(const Cell& a, const Cell& b)
{
  // std::cout << a.pose.x << ", " << b.pose.x << ", " << a.pose.y << ", " << b.pose.y << std::endl;
  return std::sqrt(std::pow(a.pose.x - b.pose.x, 2) + std::pow(a.pose.y - b.pose.y, 2));
}

// int plan(Pose start_pose, Pose goal_pose);
int plan(Map& map, const Cell& start, const Cell& goal)
{
  PROFILE_FUNCTION();
  auto compare = [](ScoredIndex a, ScoredIndex b) { return a.second > b.second; };
  map.data_[start.index].g_score = 0.;
  map.data_[start.index].f_score = distance(start, goal);

  std::priority_queue<ScoredIndex, std::vector<ScoredIndex>, decltype(compare)> open_set(compare);
  open_set.push(std::make_pair(start.index, map.data_[start.index].f_score));

  std::unordered_map<long long, Cell> came_from;

  while (!open_set.empty())
  {
    ScoredIndex current_scored_index = open_set.top();
    Cell current = map.data_.at(current_scored_index.first);
    if (current.f_score != current_scored_index.second)
    {
      open_set.pop();
      continue;
    }

    if (current.index == goal.index)
    {
      std::cout << "Found!" << std::endl;
      reconstruct_path(map.data_.at(goal.index), map.data_.at(start.index), came_from);
      return 0;
    }

    open_set.pop();

    for (const long long neighbour_index : current.neighbours)
    {
      Cell neighbour = map.data_.at(neighbour_index);
      double d = distance(current, neighbour);
      double new_score = current.g_score + d;
      if (new_score < neighbour.g_score)
      {
        came_from[neighbour.index] = current;
        map.data_[neighbour_index].g_score = new_score;
        double h = distance(neighbour, goal);
        map.data_[neighbour_index].f_score = new_score + h;
        open_set.push(std::make_pair(neighbour.index, map.data_[neighbour_index].f_score));
      }
    }
  }

  //  // Open set is empty but goal was never reached
  //  return failure
  std::cout << "Failed" << std::endl;
  return 1;
}
}  // namespace octo
