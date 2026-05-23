#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include "grid_map.h"

void NavTracker_Reset(NavTrackerState *state);
NavCommand NavTracker_Compute(const NavPose *pose,
                              const NavPath *path,
                              NavTrackerState *state,
                              float lookahead_mm,
                              float cruise_speed_mms,
                              float goal_tolerance_mm);

#endif
