#include "astar_planner/cell.hpp"

namespace octo
{
// TODO: Check how this operator overloads is linked to the std::less operator
bool Cell::operator()(const Cell* lhs, const Cell* rhs)
{
  return lhs->f_score < rhs->f_score;
}
}  // namespace octo