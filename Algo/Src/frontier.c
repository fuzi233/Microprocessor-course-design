#include "frontier.h"

#include <string.h>

static bool frontier_visited[NAV_MAP_SIZE][NAV_MAP_SIZE];
static NavCell frontier_queue[NAV_MAP_SIZE * NAV_MAP_SIZE];

static bool NavFrontier_IsFrontierCell(const NavGridMap *map, int x, int y)
{
    static const int neighbors[4][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };
    int i;

    if (!NavGridMap_IsCellTraversable(map, x, y, false))
    {
        return false;
    }

    for (i = 0; i < 4; ++i)
    {
        int nx = x + neighbors[i][0];
        int ny = y + neighbors[i][1];
        if (Nav_CellInBounds(nx, ny) && map->occupancy[ny][nx] == NAV_UNKNOWN_CELL)
        {
            return true;
        }
    }

    return false;
}

uint16_t NavFrontier_FindClusters(const NavGridMap *map,
                                  NavCell start,
                                  NavFrontierCluster *clusters,
                                  uint16_t max_clusters)
{
    uint16_t cluster_count = 0U;
    int x;
    int y;

    (void)start;
    memset(frontier_visited, 0, sizeof(frontier_visited));

    for (y = 0; y < NAV_MAP_SIZE; ++y)
    {
        for (x = 0; x < NAV_MAP_SIZE; ++x)
        {
            uint32_t head;
            uint32_t tail;
            uint32_t sum_x;
            uint32_t sum_y;
            uint16_t cluster_size;
            NavFrontierCluster cluster;

            if (frontier_visited[y][x] || !NavFrontier_IsFrontierCell(map, x, y))
            {
                continue;
            }

            head = 0U;
            tail = 0U;
            sum_x = 0U;
            sum_y = 0U;
            cluster_size = 0U;

            frontier_queue[tail++] = (NavCell){(int16_t)x, (int16_t)y};
            frontier_visited[y][x] = true;

            while (head < tail)
            {
                static const int neighbors[8][2] = {
                    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
                    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
                };
                NavCell current = frontier_queue[head++];
                int i;

                sum_x += (uint32_t)current.x;
                sum_y += (uint32_t)current.y;
                ++cluster_size;

                for (i = 0; i < 8; ++i)
                {
                    int nx = current.x + neighbors[i][0];
                    int ny = current.y + neighbors[i][1];
                    if (!Nav_CellInBounds(nx, ny) || frontier_visited[ny][nx])
                    {
                        continue;
                    }
                    if (!NavFrontier_IsFrontierCell(map, nx, ny))
                    {
                        continue;
                    }
                    frontier_visited[ny][nx] = true;
                    frontier_queue[tail++] = (NavCell){(int16_t)nx, (int16_t)ny};
                }
            }

            if (cluster_size < 3U)
            {
                continue;
            }

            cluster.centroid.x = (int16_t)(sum_x / cluster_size);
            cluster.centroid.y = (int16_t)(sum_y / cluster_size);
            cluster.representative = cluster.centroid;
            cluster.size = cluster_size;
            cluster.score = (float)cluster_size * 25.0f -
                            (float)((cluster.centroid.x - start.x) * (cluster.centroid.x - start.x) +
                                    (cluster.centroid.y - start.y) * (cluster.centroid.y - start.y));

            if (cluster_count < max_clusters)
            {
                clusters[cluster_count++] = cluster;
            }
            else
            {
                uint16_t worst_index = 0U;
                uint16_t i;
                for (i = 1U; i < max_clusters; ++i)
                {
                    if (clusters[i].score < clusters[worst_index].score)
                    {
                        worst_index = i;
                    }
                }
                if (cluster.score > clusters[worst_index].score)
                {
                    clusters[worst_index] = cluster;
                }
            }
        }
    }

    for (uint16_t i = 0U; i < cluster_count; ++i)
    {
        for (uint16_t j = (uint16_t)(i + 1U); j < cluster_count; ++j)
        {
            if (clusters[j].score > clusters[i].score)
            {
                NavFrontierCluster tmp = clusters[i];
                clusters[i] = clusters[j];
                clusters[j] = tmp;
            }
        }
    }

    return cluster_count;
}
