#include "path_tracker.h"

void NavTracker_Reset(NavTrackerState *state)
{
    state->target_index = 0U;
    state->goal_reached = false;
    state->last_target_heading_rad = 0.0f;
}

NavCommand NavTracker_Compute(const NavPose *pose,
                              const NavPath *path,
                              NavTrackerState *state,
                              float lookahead_mm,
                              float cruise_speed_mms,
                              float goal_tolerance_mm)
{
    NavCommand cmd = {0.0f, 0.0f};
    uint16_t closest_index;
    float closest_dist_sq;
    float goal_x_mm;
    float goal_y_mm;
    float goal_dx;
    float goal_dy;
    float goal_dist;

    if (path->count == 0U)
    {
        state->goal_reached = true;
        return cmd;
    }

    NavGridMap_CellToWorld(path->cells[path->count - 1U], &goal_x_mm, &goal_y_mm);
    goal_dx = goal_x_mm - pose->x_mm;
    goal_dy = goal_y_mm - pose->y_mm;
    goal_dist = sqrtf(goal_dx * goal_dx + goal_dy * goal_dy);

    if (goal_dist <= goal_tolerance_mm)
    {
        state->goal_reached = true;
        return cmd;
    }

    state->goal_reached = false;

    closest_index = state->target_index;
    if (closest_index >= path->count)
    {
        closest_index = 0U;
    }

    closest_dist_sq = 1.0e30f;
    for (uint16_t i = closest_index; i < path->count; ++i)
    {
        float px_mm;
        float py_mm;
        float dx;
        float dy;
        float dist_sq;

        NavGridMap_CellToWorld(path->cells[i], &px_mm, &py_mm);
        dx = px_mm - pose->x_mm;
        dy = py_mm - pose->y_mm;
        dist_sq = dx * dx + dy * dy;
        if (dist_sq < closest_dist_sq)
        {
            closest_dist_sq = dist_sq;
            closest_index = i;
        }
    }

    for (uint16_t i = closest_index; i < path->count; ++i)
    {
        float target_x_mm;
        float target_y_mm;
        float dx;
        float dy;
        float distance_mm;

        NavGridMap_CellToWorld(path->cells[i], &target_x_mm, &target_y_mm);
        dx = target_x_mm - pose->x_mm;
        dy = target_y_mm - pose->y_mm;
        distance_mm = sqrtf(dx * dx + dy * dy);

        if (distance_mm >= lookahead_mm || i == (uint16_t)(path->count - 1U))
        {
            float alpha;
            float curvature;
            float linear;

            state->target_index = i;
            state->last_target_heading_rad = atan2f(dy, dx);
            alpha = Nav_NormalizeAngle(state->last_target_heading_rad - pose->theta_rad);

            linear = cruise_speed_mms;
            if (fabsf(alpha) > 1.0f)
            {
                linear *= 0.35f;
            }
            else if (fabsf(alpha) > 0.6f)
            {
                linear *= 0.6f;
            }
            if (goal_dist < lookahead_mm * 2.0f)
            {
                linear *= Nav_ClampF(goal_dist / (lookahead_mm * 2.0f), 0.25f, 1.0f);
            }

            curvature = 2.0f * sinf(alpha) / Nav_ClampF(distance_mm, 1.0f, lookahead_mm * 2.0f);
            cmd.linear_vel_mms = linear;
            cmd.angular_vel_rads = Nav_ClampF(linear * curvature, -1.8f, 1.8f);
            return cmd;
        }
    }

    return cmd;
}
