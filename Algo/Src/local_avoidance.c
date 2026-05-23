#include "local_avoidance.h"

void NavAvoidance_BuildSectors(const NavScan *scan,
                               float block_distance_mm,
                               NavSectorScan *sector_scan)
{
    uint16_t i;
    float sector_width = 2.0f * (float)M_PI / (float)NAV_SECTOR_COUNT;

    for (i = 0U; i < NAV_SECTOR_COUNT; ++i)
    {
        sector_scan->sector_min_distance_mm[i] = scan->max_range_mm;
        sector_scan->sector_blocked[i] = false;
    }

    for (i = 0U; i < scan->count; ++i)
    {
        int sector_index;
        float normalized;

        if (!scan->beams[i].valid)
        {
            continue;
        }

        normalized = Nav_NormalizeAngle(scan->beams[i].angle_rad);
        sector_index = (int)((normalized + (float)M_PI) / sector_width);
        if (sector_index < 0)
        {
            sector_index = 0;
        }
        if (sector_index >= NAV_SECTOR_COUNT)
        {
            sector_index = NAV_SECTOR_COUNT - 1;
        }

        if (scan->beams[i].distance_mm < sector_scan->sector_min_distance_mm[sector_index])
        {
            sector_scan->sector_min_distance_mm[sector_index] = scan->beams[i].distance_mm;
        }
    }

    for (i = 0U; i < NAV_SECTOR_COUNT; ++i)
    {
        sector_scan->sector_blocked[i] =
            (sector_scan->sector_min_distance_mm[i] < block_distance_mm);
    }
}

NavCommand NavAvoidance_Apply(const NavPose *pose,
                              const NavScan *scan,
                              const NavCommand *base_cmd,
                              float desired_heading_rad,
                              float block_distance_mm,
                              float emergency_distance_mm)
{
    NavCommand adjusted = *base_cmd;
    NavSectorScan sectors;
    float desired_local;
    float front_min;
    uint16_t front_center;

    (void)pose;

    NavAvoidance_BuildSectors(scan, block_distance_mm, &sectors);

    front_center = NAV_SECTOR_COUNT / 2U;
    front_min = sectors.sector_min_distance_mm[front_center];
    if (sectors.sector_min_distance_mm[front_center - 1U] < front_min)
    {
        front_min = sectors.sector_min_distance_mm[front_center - 1U];
    }
    if (sectors.sector_min_distance_mm[front_center + 1U] < front_min)
    {
        front_min = sectors.sector_min_distance_mm[front_center + 1U];
    }

    if (front_min >= block_distance_mm)
    {
        return adjusted;
    }

    desired_local = Nav_NormalizeAngle(desired_heading_rad - pose->theta_rad);

    if (front_min < emergency_distance_mm)
    {
        adjusted.linear_vel_mms = 0.0f;
    }
    else
    {
        adjusted.linear_vel_mms = Nav_ClampF(base_cmd->linear_vel_mms, 0.0f, 120.0f);
    }

    {
        float sector_width = 2.0f * (float)M_PI / (float)NAV_SECTOR_COUNT;
        float best_score = -1.0e30f;
        int best_sector = -1;

        for (int i = 0; i < NAV_SECTOR_COUNT; ++i)
        {
            float sector_center;
            float heading_error;
            float score;

            if (sectors.sector_blocked[i])
            {
                continue;
            }

            sector_center = -(float)M_PI + ((float)i + 0.5f) * sector_width;
            heading_error = fabsf(Nav_NormalizeAngle(sector_center - desired_local));
            score = sectors.sector_min_distance_mm[i] - heading_error * 900.0f;

            if (score > best_score)
            {
                best_score = score;
                best_sector = i;
            }
        }

        if (best_sector < 0)
        {
            adjusted.linear_vel_mms = 0.0f;
            adjusted.angular_vel_rads = (desired_local >= 0.0f) ? 0.9f : -0.9f;
            return adjusted;
        }

        {
            float steer_angle = -(float)M_PI + ((float)best_sector + 0.5f) * sector_width;
            adjusted.angular_vel_rads = Nav_ClampF(steer_angle * 1.8f, -1.5f, 1.5f);
        }
    }

    return adjusted;
}
