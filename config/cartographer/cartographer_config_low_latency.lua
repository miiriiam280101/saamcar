-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1., --everything other than 1. drops input data
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 12           -- adas2019 car has 6 cores 12 threads

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10 -- commend this line for default settings
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false

POSE_GRAPH.constraint_builder.max_constraint_distance = 5. -- 5 
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.65

POSE_GRAPH.optimization_problem.huber_scale = 1e2

LOCAL_SLAM.submaps.resolution = 0.1

-- this is only for localisation

return options



-- -- GLOBAL SLAM TUNING -- -- see: https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html#low-latency
-- decrease optimize_every_n_nodes
-- increase MAP_BUILDER.num_background_threads up to the number of cores
-- decrease global_sampling_ratio
-- decrease constraint_builder.sampling_ratio
-- increase constraint_builder.min_score
-- for the adaptive voxel filter(s), decrease .min_num_points, .max_range, increase .max_length
-- increase voxel_filter_size, submaps.resolution, decrease submaps.num_range_data
-- decrease search windows sizes, .linear_xy_search_window, .linear_z_search_window, .angular_search_window
-- increase global_constraint_search_after_n_seconds
-- decrease max_num_iterations

-- -- LOCAL SLAM TUNING -- --
-- increase voxel_filter_size
-- increase submaps.resolution
-- for the adaptive voxel filter(s), decrease .min_num_points, .max_range, increase .max_length
-- decrease max_range (especially if data is noisy)
-- decrease submaps.num_range_data

