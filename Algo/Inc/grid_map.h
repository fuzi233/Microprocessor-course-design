#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "nav_types.h"

/* 查找表版三角函数 (替代 math.h, 避免 FPU 库崩溃) */
float trig_cos(float rad);
float trig_sin(float rad);
float trig_atan2(float y, float x);

void NavGridMap_Init(NavGridMap *map);
bool NavGridMap_WorldToCell(float x_mm, float y_mm, NavCell *cell);
void NavGridMap_CellToWorld(NavCell cell, float *x_mm, float *y_mm);
void NavGridMap_InsertScan(NavGridMap *map, const NavPose *pose, const NavScan *scan);
void NavGridMap_Inflate(NavGridMap *map, uint8_t radius_cells);
bool NavGridMap_IsCellTraversable(const NavGridMap *map, int x, int y, bool use_inflated);
bool NavGridMap_IsPathBlocked(const NavGridMap *map, const NavPath *path, uint16_t from_index);
uint32_t NavGridMap_CountKnownFree(const NavGridMap *map);

#endif
