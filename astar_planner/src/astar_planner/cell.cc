#include "astar_planner/cell.hpp"

namespace octo
{
Cell::Cell()
{
}

Cell::Cell(double x, double y, bool occupied, int index) : pose_{ x, y }, occupied_(occupied), index_(index)
{
}

// TODO: Check how this operator overloads is linked to the std::less operator
bool Cell::operator()(const Cell* lhs, const Cell* rhs)
{
  return lhs->f_score_ < rhs->f_score_;
}

}  // namespace octo