#pragma once
#include "mppi.h"

void build_lidar_sectors(void);

float calc_goal_cost(const MPPI_State *state);
float calc_heading_cost(const MPPI_State *state);
float calc_sector_obstacle_cost(const MPPI_State *pred_state,
                                const MPPI_State *current_state);
float calc_input_cost(const MPPI_Input *input);
float calc_smooth_cost(const MPPI_Input *input, const MPPI_Input *prev_input);

const float *get_sector_distance_array(void);
const float *get_sector_angle_array(void);

