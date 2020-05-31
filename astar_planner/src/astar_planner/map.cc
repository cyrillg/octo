#include "astar_planner/map.hpp"

namespace octo
{
void Map::find_neighbours(int i, int j, std::vector<std::pair<int, int>>& neighbours)
{
  // Inside
  if (i > 0 && i < this->height_ - 1 && j > 0 && j < this->width_ - 1)
  {
    neighbours.insert(neighbours.end(), { top_left, top, top_right, right, bottom_right, bottom, bottom_left, left });
    return;
  }

  // Sides
  if (i == 0 && j != 0)
  {
    neighbours.insert(neighbours.end(), { left, top_left, top, top_right, right });
  }
  else if (j == 0 && i != 0)
  {
    neighbours.insert(neighbours.end(), { top, top_right, right, bottom_right, bottom });
  }
  else if (j == this->width_ - 1 && i != 0 && i != this->height_ - 1)
  {
    neighbours.insert(neighbours.end(), { bottom, bottom_left, left, top_left, top });
  }
  else if (i == this->height_ - 1 && j != 0 && j != this->width_ - 1)
  {
    neighbours.insert(neighbours.end(), { right, bottom_right, bottom, bottom_left, left });
  }
  else
  {
    // Corners
    if (i == 0 && j == 0)
    {
      neighbours.insert(neighbours.end(), { top, top_right, right });
    }
    else if (i == this->height_ - 1 && j == 0)
    {
      neighbours.insert(neighbours.end(), { right, bottom_right, bottom });
    }
    else if (i == this->height_ - 1 && j == this->width_ - 1)
    {
      neighbours.insert(neighbours.end(), { bottom, bottom_left, left });
    }
    else if (i == 0 && j == this->width_ - 1)
    {
      neighbours.insert(neighbours.end(), { left, top_left, top });
    }
  }
}

void Map::print_map()
{
  PROFILE_FUNCTION();
  for (int i = this->height_ - 1; i > -1; i--)
  {
    std::stringstream ss1;
    std::stringstream ss2;
    for (int j = 0; j < this->width_; j++)
    {
      ss1 << " " << this->data_.at(static_cast<long long>(j) + static_cast<long long>(i) * this->height_).g_score_;
      ss2 << " " << this->data_.at(static_cast<long long>(j) + static_cast<long long>(i) * this->height_).index_;
    }
    std::cout << ss1.str() << " | " << ss2.str() << std::endl;
  }
};

}  // namespace octo