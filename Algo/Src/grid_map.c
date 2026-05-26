#include "grid_map.h"

#include <string.h>

/* ========================================================================
 * 简易三角函数查找表 (替代 cosf/sinf, 避免 FPU 数学库崩溃)
 * 分辨率: 256 等分 / 2π, 覆盖 0 ~ 2π
 * ======================================================================== */

#define TRIG_TABLE_BITS  8U
#define TRIG_TABLE_SIZE  (1U << TRIG_TABLE_BITS)  /* 256 */
#define TRIG_TABLE_MASK  (TRIG_TABLE_SIZE - 1U)

static const float g_trig_sin[TRIG_TABLE_SIZE] = {
     0.000000f,  0.024541f,  0.049068f,  0.073565f,  0.098017f,  0.122411f,
     0.146730f,  0.170962f,  0.195090f,  0.219101f,  0.242980f,  0.266713f,
     0.290285f,  0.313682f,  0.336890f,  0.359895f,  0.382683f,  0.405241f,
     0.427555f,  0.449611f,  0.471397f,  0.492898f,  0.514103f,  0.534998f,
     0.555570f,  0.575808f,  0.595699f,  0.615232f,  0.634393f,  0.653173f,
     0.671559f,  0.689541f,  0.707107f,  0.724247f,  0.740951f,  0.757209f,
     0.773010f,  0.788346f,  0.803208f,  0.817585f,  0.831470f,  0.844854f,
     0.857729f,  0.870087f,  0.881921f,  0.893224f,  0.903989f,  0.914210f,
     0.923880f,  0.932993f,  0.941544f,  0.949528f,  0.956940f,  0.963776f,
     0.970031f,  0.975702f,  0.980785f,  0.985278f,  0.989177f,  0.992480f,
     0.995185f,  0.997290f,  0.998795f,  0.999699f,  1.000000f,  0.999699f,
     0.998795f,  0.997290f,  0.995185f,  0.992480f,  0.989177f,  0.985278f,
     0.980785f,  0.975702f,  0.970031f,  0.963776f,  0.956940f,  0.949528f,
     0.941544f,  0.932993f,  0.923880f,  0.914210f,  0.903989f,  0.893224f,
     0.881921f,  0.870087f,  0.857729f,  0.844854f,  0.831470f,  0.817585f,
     0.803208f,  0.788346f,  0.773010f,  0.757209f,  0.740951f,  0.724247f,
     0.707107f,  0.689541f,  0.671559f,  0.653173f,  0.634393f,  0.615232f,
     0.595699f,  0.575808f,  0.555570f,  0.534998f,  0.514103f,  0.492898f,
     0.471397f,  0.449611f,  0.427555f,  0.405241f,  0.382683f,  0.359895f,
     0.336890f,  0.313682f,  0.290285f,  0.266713f,  0.242980f,  0.219101f,
     0.195090f,  0.170962f,  0.146730f,  0.122411f,  0.098017f,  0.073565f,
     0.049068f,  0.024541f,  0.000000f, -0.024541f, -0.049068f, -0.073565f,
    -0.098017f, -0.122411f, -0.146730f, -0.170962f, -0.195090f, -0.219101f,
    -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f,
    -0.514103f, -0.534998f, -0.555570f, -0.575808f, -0.595699f, -0.615232f,
    -0.634393f, -0.653173f, -0.671559f, -0.689541f, -0.707107f, -0.724247f,
    -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f,
    -0.903989f, -0.914210f, -0.923880f, -0.932993f, -0.941544f, -0.949528f,
    -0.956940f, -0.963776f, -0.970031f, -0.975702f, -0.980785f, -0.985278f,
    -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f,
    -0.989177f, -0.985278f, -0.980785f, -0.975702f, -0.970031f, -0.963776f,
    -0.956940f, -0.949528f, -0.941544f, -0.932993f, -0.923880f, -0.914210f,
    -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f,
    -0.740951f, -0.724247f, -0.707107f, -0.689541f, -0.671559f, -0.653173f,
    -0.634393f, -0.615232f, -0.595699f, -0.575808f, -0.555570f, -0.534998f,
    -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f,
    -0.242980f, -0.219101f, -0.195090f, -0.170962f, -0.146730f, -0.122411f,
    -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

/* 角度归一化到 uint8_t 索引 [0, 255) */
static uint8_t trig_rad_to_index(float rad)
{
    float scaled = rad * (float)TRIG_TABLE_SIZE / (2.0f * (float)M_PI);
    int idx = (int)scaled;
    /* 处理负角度和超范围 */
    idx = idx % (int)TRIG_TABLE_SIZE;
    if (idx < 0) idx += (int)TRIG_TABLE_SIZE;
    return (uint8_t)idx;
}

float trig_cos(float rad)
{
    return g_trig_sin[(trig_rad_to_index(rad) + (TRIG_TABLE_SIZE / 4U)) & TRIG_TABLE_MASK];
}

float trig_sin(float rad)
{
    return g_trig_sin[trig_rad_to_index(rad)];
}

/* atan2 多项式近似 (误差 < 0.001 rad) */
float trig_atan2(float y, float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;
    float ay = (y > 0.0f) ? y : -y;
    float ax = (x > 0.0f) ? x : -x;
    float a;
    if (ax > ay) {
        float r = ay / ax;
        float r2 = r * r;
        a = r * (0.9998660f - r2 * (0.3302995f - r2 * 0.1801410f));
    } else {
        float r = ax / ay;
        float r2 = r * r;
        a = 1.5707963f - r * (0.9998660f - r2 * (0.3302995f - r2 * 0.1801410f));
    }
    if (x < 0.0f) a = 3.1415927f - a;
    if (y < 0.0f) a = -a;
    return a;
}

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
        hit_x_mm = pose->x_mm + scan->beams[i].distance_mm * trig_cos(beam_theta);
        hit_y_mm = pose->y_mm + scan->beams[i].distance_mm * trig_sin(beam_theta);

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
