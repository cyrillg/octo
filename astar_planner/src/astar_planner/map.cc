#include "astar_planner/map.hpp"

namespace octo
{
std::vector<std::pair<int, int>> Map::find_neighbours(int i,
                                                      int j) const  // TODO: Create hash table with custom hash function
                                                                    // to allow look-up instead of branching
{
  // Inside
  if (i > 0 && i < this->height_ - 1 && j > 0 && j < this->width_ - 1)
  {
    return { top_left, top, top_right, right, bottom_right, bottom, bottom_left, left };
  }

  // Sides
  if (i == 0 && j != 0)
  {
    return { left, top_left, top, top_right, right };
  }
  if (j == 0 && i != 0)
  {
    return { top, top_right, right, bottom_right, bottom };
  }
  if (j == this->width_ - 1 && i != 0 && i != this->height_ - 1)
  {
    return { bottom, bottom_left, left, top_left, top };
  }
  if (i == this->height_ - 1 && j != 0 && j != this->width_ - 1)
  {
    return { right, bottom_right, bottom, bottom_left, left };
  }

  // Corners
  if (i == 0 && j == 0)
  {
    return { top, top_right, right };
  }
  if (i == this->height_ - 1 && j == 0)
  {
    return { right, bottom_right, bottom };
  }
  if (i == this->height_ - 1 && j == this->width_ - 1)
  {
    return { bottom, bottom_left, left };
  }
  if (i == 0 && j == this->width_ - 1)
  {
    return { left, top_left, top };
  }

  return {};
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
      ss1 << " " << this->data_.at(static_cast<long long>(j) + static_cast<long long>(i) * this->height_).g_score;
      ss2 << " " << this->data_.at(static_cast<long long>(j) + static_cast<long long>(i) * this->height_).index;
    }
    std::cout << ss1.str() << " | " << ss2.str() << std::endl;
  }
};

}  // namespace octo