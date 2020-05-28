#include "astar_planner/planner.hpp"

// a hack square root calculation using simple operations
namespace octo
{
Cell::Cell(double x, double y, bool occupied, int index) : pose_{ x, y }, occupied_(occupied), index_(index)
{
}

// TODO: Check how this operator overloads is linked to the std::less operator
bool Cell::operator()(const Cell* lhs, const Cell* rhs)
{
  return lhs->f_score_ < rhs->f_score_;
}

void find_neighbours(const Map& map, int i, int j, std::vector<std::pair<int, int>>& neighbours)
{
  std::pair<int, int> top_left = std::make_pair(1, -1);
  std::pair<int, int> top = std::make_pair(1, 0);
  std::pair<int, int> top_right = std::make_pair(1, 1);
  std::pair<int, int> right = std::make_pair(0, 1);
  std::pair<int, int> bottom_right = std::make_pair(-1, 1);
  std::pair<int, int> bottom = std::make_pair(-1, 0);
  std::pair<int, int> bottom_left = std::make_pair(-1, -1);
  std::pair<int, int> left = std::make_pair(0, -1);

  // Corners
  if (i == 0 && j == 0)
  {
    neighbours.emplace_back(top);
    neighbours.emplace_back(top_right);
    neighbours.emplace_back(right);
  }
  else if (i == map.height_ - 1 && j == 0)
  {
    neighbours.emplace_back(right);
    neighbours.emplace_back(bottom_right);
    neighbours.emplace_back(bottom);
  }
  else if (i == map.height_ - 1 && j == map.width_ - 1)
  {
    neighbours.emplace_back(bottom);
    neighbours.emplace_back(bottom_left);
    neighbours.emplace_back(left);
  }
  else if (i == 0 && j == map.width_ - 1)
  {
    neighbours.emplace_back(left);
    neighbours.emplace_back(top_left);
    neighbours.emplace_back(top);
  }
  else
  {
    // Sides
    if (i == 0)
    {
      neighbours.emplace_back(left);
      neighbours.emplace_back(top_left);
      neighbours.emplace_back(top);
      neighbours.emplace_back(top_right);
      neighbours.emplace_back(right);
    }
    else if (j == 0)
    {
      neighbours.emplace_back(top);
      neighbours.emplace_back(top_right);
      neighbours.emplace_back(right);
      neighbours.emplace_back(bottom_right);
      neighbours.emplace_back(bottom);
    }
    else if (j == map.width_ - 1)
    {
      neighbours.emplace_back(bottom);
      neighbours.emplace_back(bottom_left);
      neighbours.emplace_back(left);
      neighbours.emplace_back(top_left);
      neighbours.emplace_back(top);
    }
    else if (i == map.height_ - 1)
    {
      neighbours.emplace_back(right);
      neighbours.emplace_back(bottom_right);
      neighbours.emplace_back(bottom);
      neighbours.emplace_back(bottom_left);
      neighbours.emplace_back(left);
    }
    // Inside
    else
    {
      neighbours.emplace_back(top_left);
      neighbours.emplace_back(top);
      neighbours.emplace_back(top_right);
      neighbours.emplace_back(right);
      neighbours.emplace_back(bottom_right);
      neighbours.emplace_back(bottom);
      neighbours.emplace_back(bottom_left);
      neighbours.emplace_back(left);
    }
  }
}

Map generate_map(int width, int height, double resolution, Pose origin)
{
  Map map;
  map.width_ = width;
  map.height_ = height;
  map.data_.reserve(static_cast<long long>(width) * height);

  if (width < 2 || height < 2)
  {
    std::cout << "Map must at least be 2x2" << std::endl;
    exit(1);
  }

  double x;
  double y;
  for (int i = 0; i < height; i++)
  {
    y = origin.y_;
    for (int j = 0; j < width; j++)
    {
      x = origin.x_ + j * resolution;
      y = origin.y_ + i * resolution;
      std::shared_ptr<Cell> cell = std::make_shared<Cell>(x, y, false, j + i * height);
      map.data_.emplace_back(cell);
    }
  }
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      std::shared_ptr<Cell> current_cell =
          map.data_.at(static_cast<long long>(j) + static_cast<long long>(i) * map.height_);

      std::vector<std::pair<int, int>> neighbours;
      find_neighbours(map, i, j, neighbours);

      for (auto neighbour : neighbours)
      {
        long long neighbour_index =
            j + static_cast<long long>(neighbour.second) + (static_cast<long long>(neighbour.first) + i) * height;
        current_cell->neighbours_.push_back(map.data_.at(neighbour_index));
      }
    }
  }

  return map;
}

double distance(const Cell& a, const Cell& b)
{
  return std::sqrt(std::pow(a.pose_.x_ - b.pose_.x_, 2) + std::pow(a.pose_.y_ - b.pose_.y_, 2));
}

// int plan(Pose start_pose, Pose goal_pose);
int plan()
{
  int index_start = 0;

  //// The set of discovered nodes that may need to be (re-)expanded.
  //// Initially, only the start node is known.
  //// This is usually implemented as a min-heap or priority queue rather than a hash-set.
  // openSet:= {start}
  Map map = generate_map(3, 3, 1, { 0, 0 });

  std::priority_queue<std::shared_ptr<Cell>, std::vector<std::shared_ptr<Cell>>, std::less<std::shared_ptr<Cell>>>
      open_set;
  open_set.push(map.data_.at(index_start));

  Cell goal = *map.data_.back();

  //  // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
  //  // to n currently known.
  //  cameFrom := an empty map
  std::unordered_map<int, Cell> came_from;

  //  // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
  //  gScore := map with default value of Infinity
  //  gScore[start] := 0
  map.data_.at(index_start)->g_score_ = 0.;

  //  // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
  //  // how short a path from start to finish can be if it goes through n.
  //  fScore := map with default value of Infinity
  //  fScore[start] := h(start)
  map.data_.at(index_start)->f_score_ = distance(*map.data_.at(index_start), goal);

  //  while openSet is not empty
  while (open_set.size())
  {
    // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
    // current := the node in openSet having the lowest fScore[] value
    //      if current = goal
    //          return reconstruct_path(cameFrom, current)

    Cell current = *open_set.top();
    if (true)
    {
      std::cout << "Found!" << std::endl;
    }
    //      openSet.Remove(current)
    open_set.pop();
    //      for each neighbor of current
    for (const auto neighbour : current.neighbours_)
    {
      //          // d(current,neighbor) is the weight of the edge from current to neighbor
      //          // tentative_gScore is the distance from start to the neighbor through current
      //          tentative_gScore := gScore[current] + d(current, neighbor)
      double d = distance(current, *neighbour);
      double new_score = current.g_score_ + d;
      //          if tentative_gScore < gScore[neighbor]
      //              // This path to neighbor is better than any previous one. Record it!
      //              cameFrom[neighbor] := current
      //              gScore[neighbor] := tentative_gScore
      //              fScore[neighbor] := gScore[neighbor] + h(neighbor)
      //              if neighbor not in openSet
      //                  openSet.add(neighbor)
      if (new_score < neighbour->g_score_)
      {
        came_from[neighbour->index_] = current;
        neighbour->g_score_ = new_score;
        neighbour->f_score_ = new_score + distance(*neighbour, goal);
        // if TODO
      }
    }
    break;
  }

  //  // Open set is empty but goal was never reached
  //  return failure
  return 0;
}
}  // namespace octo
