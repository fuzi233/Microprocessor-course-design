#ifndef LOCAL_AVOIDANCE_H
#define LOCAL_AVOIDANCE_H

#include "nav_types.h"

void NavAvoidance_BuildSectors(const NavScan *scan,
                               float block_distance_mm,
                               NavSectorScan *sector_scan);

NavCommand NavAvoidance_Apply(const NavPose *pose,
                              const NavScan *scan,
                              const NavCommand *base_cmd,
                              float desired_heading_rad,
                              float block_distance_mm,
                              float emergency_distance_mm);

#endif
