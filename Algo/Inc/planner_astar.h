#ifndef PLANNER_ASTAR_H
#define PLANNER_ASTAR_H

#include "grid_map.h"

bool NavPlanner_AStar(const NavGridMap *map, NavCell start, NavCell goal, NavPath *path);
void NavPlanner_CompressPath(NavPath *path);

#endif
