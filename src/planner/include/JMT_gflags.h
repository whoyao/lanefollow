//
// Created by huyao on 2018/9/27.
// Email: hooyao@sjtu.edu.cn
//

#ifndef PROJECT_JMT_GFLAGS_H
#define PROJECT_JMT_GFLAGS_H

#include "gflags/gflags.h"


// no lane change
DECLARE_double(car_default_d);

// safaty
DECLARE_double(half_car_length);
DECLARE_double(half_car_width);
DECLARE_double(head_offset);
DECLARE_double(origin_to_center);
DECLARE_double(bumper_to_front);

// end sample
DECLARE_int32(num_velocity_sample);
DECLARE_double(trajectory_time_length);
DECLARE_double(min_velocity_sample_gap);
DECLARE_double(polynomial_minimal_param);
DECLARE_double(ratio_safe_dis);
DECLARE_double(radius_safe_dis);
DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);


// smoother
DECLARE_int32(clothoide_n_lookahead);
//DECLARE_double(piecewise_length);

// trajectory generator
DECLARE_double(lattice_epsilon);
DECLARE_double(trajectory_time_resolution);

// evaluator
DECLARE_double(lattice_stop_buffer);
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);
DECLARE_double(lateral_acceleration_bound);
DECLARE_double(weight_target_speed);
DECLARE_double(weight_dist_travelled);
DECLARE_double(decision_horizon);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(force_route_dis);
DECLARE_double(longitudinal_jerk_upper_bound);

// weight
DECLARE_double(weight_static_collision);
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_route_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(weight_consistency);
DECLARE_double(weight_change_distance);
/*

// parameters for trajectory planning
DECLARE_double(planning_upper_speed_limit);

DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);


// lattice planner
DECLARE_double(lattice_epsilon);
DECLARE_double(default_cruise_speed);

DECLARE_double(trajectory_time_resolution);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(lateral_acceleration_bound);
DECLARE_double(decision_horizon);
DECLARE_uint32(num_velocity_sample);
DECLARE_bool(enable_backup_trajectory);
DECLARE_double(backup_trajectory_cost);

DECLARE_double(lon_collision_buffer);
DECLARE_double(lat_collision_buffer);
DECLARE_uint32(num_sample_follow_per_timestamp);

DECLARE_bool(lateral_optimization);
DECLARE_double(weight_lateral_offset);
DECLARE_double(weight_lateral_derivative);
DECLARE_double(weight_lateral_second_order_derivative);
DECLARE_double(weight_lateral_obstacle_distance);
DECLARE_double(lateral_third_order_derivative_max);

// Lattice Evaluate Parameters
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(cost_non_priority_reference_line);
DECLARE_double(weight_same_side_offset);
DECLARE_double(weight_opposite_side_offset);
DECLARE_double(weight_dist_travelled);
DECLARE_double(weight_target_speed);
DECLARE_double(lat_offset_bound);
DECLARE_double(lon_collision_yield_buffer);
DECLARE_double(lon_collision_overtake_buffer);
DECLARE_double(lon_collision_cost_std);
DECLARE_double(default_lon_buffer);
DECLARE_double(time_min_density);
DECLARE_double(comfort_acceleration_factor);

DECLARE_double(lattice_stop_buffer);
DECLARE_double(max_s_lateral_optimization);
DECLARE_double(default_delta_s_lateral_optimization);
DECLARE_double(bound_buffer);
DECLARE_double(nudge_buffer);

DECLARE_bool(use_planning_fallback);
DECLARE_double(fallback_total_time);
DECLARE_double(fallback_time_unit);
DECLARE_double(polynomial_speed_fallback_velocity);

// navigation mode
DECLARE_double(navigation_fallback_cruise_time);

// control whether to stitch last trajectory to current plan trajectory
DECLARE_bool(enable_stitch_last_trajectory);
DECLARE_bool(enable_planning_pad_msg);
*/

#endif //PROJECT_JMT_GFLAGS_H
