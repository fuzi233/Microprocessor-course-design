#include "grid_map.h"

#include <string.h>

static void NavGridMap_SetFreeIfUnknownOrFree(NavGridMap *map, int x, int y)
{
    int8_t *cell;

    if (!Nav_CellInBounds(x, y))
    {
        return;
    }

    cell = &map->occupancy[y][x];
    if (*cell != NAV_OCCUPIED_CELL)
    {
        *cell = NAV_FREE_CELL;
    }
}

static void NavGridMap_SetOccupied(NavGridMap *map, int x, int y)
{
    if (!Nav_CellInBounds(x, y))
    {
        return;
    }

    map->occupancy[y][x] = NAV_OCCUPIED_CELL;
}

static void NavGridMap_DrawRayFree(NavGridMap *map, NavCell start, NavCell end_exclusive)
{
    int x0 = start.x;
    int y0 = start.y;
    int x1 = end_exclusive.x;
    int y1 = end_exclusive.y;
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? -(y1 - y0) : -(y0 - y1);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while ((x0 != x1) || (y0 != y1))
    {
        NavGridMap_SetFreeIfUnknownOrFree(map, x0, y0);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        if (2 * err >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (2 * err <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void NavGridMap_Init(NavGridMap *map)
{
    uint16_t x;
    uint16_t y;

    for (y = 0; y < NAV_MAP_SIZE; ++y)
    {
        for (x = 0; x < NAV_MAP_SIZE; ++x)
        {
            map->occupancy[y][x] = NAV_UNKNOWN_CELL;
            map->inflated[y][x] = 0U;
        }
    }
}

bool NavGridMap_WorldToCell(float x_mm, float y_mm, NavCell *cell)
{
    const float half_extent_mm = (NAV_MAP_SIZE * NAV_CELL_SIZE_MM) * 0.5f;
    int cell_x;
    int cell_y;

    cell_x = (int)((x_mm + half_extent_mm) / NAV_CELL_SIZE_MM);
    cell_y = (int)((y_mm + half_extent_mm) / NAV_CELL_SIZE_MM);

    if (!Nav_CellInBounds(cell_x, cell_y))
    {
        return false;
    }

    cell->x = (int16_t)cell_x;
    cell->y = (int16_t)cell_y;
    return true;
}

void NavGridMap_CellToWorld(NavCell cell, float *x_mm, float *y_mm)
{
    const float half_extent_mm = (NAV_MAP_SIZE * NAV_CELL_SIZE_MM) * 0.5f;

    *x_mm = ((float)cell.x + 0.5f) * NAV_CELL_SIZE_MM - half_extent_mm;
    *y_mm = ((float)cell.y + 0.5f) * NAV_CELL_SIZE_MM - half_extent_mm;
}

void NavGridMap_InsertScan(NavGridMap *map, const NavPose *pose, const NavScan *scan)
{
    NavCell robot_cell;
    uint16_t i;

    if (!NavGridMap_WorldToCell(pose->x_mm, pose->y_mm, &robot_cell))
    {
        return;
    }

    NavGridMap_SetFreeIfUnknownOrFree(map, robot_cell.x, robot_cell.y);

    for (i = 0; i < scan->count; ++i)
    {
        NavCell hit_cell;
        float beam_theta;
        float hit_x_mm;
        float hit_y_mm;
        bool mark_hit_occupied;

        if (!scan->beams[i].valid || scan->beams[i].distance_mm <= 1.0f)
        {
            continue;
        }

        beam_theta = pose->theta_rad + scan->beams[i].angle_rad;
        hit_x_mm = pose->x_mm + scan->beams[i].distance_mm * cosf(beam_theta);
        hit_y_mm = pose->y_mm + scan->beams[i].distance_mm * sinf(beam_theta);

        if (!NavGridMap_WorldToCell(hit_x_mm, hit_y_mm, &hit_cell))
        {
            continue;
        }

        mark_hit_occupied = (scan->beams[i].distance_mm < (scan->max_range_mm - NAV_CELL_SIZE_MM));
        if (mark_hit_occupied)
        {
            NavCell free_until = hit_cell;
            if (free_until.x != robot_cell.x || free_until.y != robot_cell.y)
            {
                if (free_until.x > robot_cell.x)
                {
                    free_until.x -= 1;
                }
                else if (free_until.x < robot_cell.x)
                {
                    free_until.x += 1;
                }

                if (free_until.y > robot_cell.y)
                {
                    free_until.y -= 1;
                }
                else if (free_until.y < robot_cell.y)
                {
                    free_until.y += 1;
                }
            }
            NavGridMap_DrawRayFree(map, robot_cell, free_until);
            NavGridMap_SetOccupied(map, hit_cell.x, hit_cell.y);
        }
        else
        {
            NavGridMap_DrawRayFree(map, robot_cell, hit_cell);
        }
    }
}

void NavGridMap_Inflate(NavGridMap *map, uint8_t radius_cells)
{
    int x;
    int y;

    memset(map->inflated, 0, sizeof(map->inflated));

    for (y = 0; y < NAV_MAP_SIZE; ++y)
    {
        for (x = 0; x < NAV_MAP_SIZE; ++x)
        {
            if (map->occupancy[y][x] != NAV_OCCUPIED_CELL)
            {
                continue;
            }

            for (int oy = -(int)radius_cells; oy <= (int)radius_cells; ++oy)
            {
                for (int ox = -(int)radius_cells; ox <= (int)radius_cells; ++ox)
                {
                    int nx = x + ox;
                    int ny = y + oy;
                    if (!Nav_CellInBounds(nx, ny))
                    {
                        continue;
                    }
                    if ((ox * ox + oy * oy) <= (radius_cells * radius_cells))
                    {
                        map->inflated[ny][nx] = 1U;
                    }
                }
            }
        }
    }
}

bool NavGridMap_IsCellTraversable(const NavGridMap *map, int x, int y, bool use_inflated)
{
    if (!Nav_CellInBounds(x, y))
    {
        return false;
    }

    if (map->occupancy[y][x] == NAV_OCCUPIED_CELL)
    {
        return false;
    }

    if (use_inflated && map->inflated[y][x] != 0U)
    {
        return false;
    }

    return map->occupancy[y][x] == NAV_FREE_CELL;
}

bool NavGridMap_IsPathBlocked(const NavGridMap *map, const NavPath *path, uint16_t from_index)
{
    uint16_t i;

    if (from_index >= path->count)
    {
        return false;
    }

    for (i = from_index; i < path->count; ++i)
    {
        if (!NavGridMap_IsCellTraversable(map, path->cells[i].x, path->cells[i].y, true))
        {
            return true;
        }
    }

    return false;
}

uint32_t NavGridMap_CountKnownFree(const NavGridMap *map)
{
    uint32_t count = 0U;
    int x;
    int y;

    for (y = 0; y < NAV_MAP_SIZE; ++y)
    {
        for (x = 0; x < NAV_MAP_SIZE; ++x)
        {
            if (map->occupancy[y][x] == NAV_FREE_CELL)
            {
                ++count;
            }
        }
    }

    return count;
}
