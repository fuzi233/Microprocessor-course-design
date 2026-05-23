#ifndef FRONTIER_H
#define FRONTIER_H

#include "grid_map.h"

uint16_t NavFrontier_FindClusters(const NavGridMap *map,
                                  NavCell start,
                                  NavFrontierCluster *clusters,
                                  uint16_t max_clusters);

#endif
