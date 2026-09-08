"""ROS Humble parameters: no vendor transforms, FAST-LIVO2 or smoother."""

import json


def parameters(c, map_yaml=""):
    scan, odom = c["scan_topic"], c["odom_topic"]
    mf, of, bf = c["map_frame"], c["odom_frame"], c["base_frame"]
    obstacle = {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
                "observation_sources": "scan",
                "scan": {"topic": scan, "data_type": "LaserScan", "marking": True,
                         "clearing": True, "inf_is_valid": True,
                         "obstacle_max_range": 8.0, "raytrace_max_range": 10.0}}
    inflation = {"plugin": "nav2_costmap_2d::InflationLayer",
                 "inflation_radius": c["inflation_radius"], "cost_scaling_factor": 3.0}
    common = {"use_sim_time": False, "robot_base_frame": bf,
              "resolution": c["resolution"], "footprint": json.dumps(c["footprint"]),
              "footprint_padding": 0.02, "transform_tolerance": 0.2,
              "always_send_full_costmap": True, "obstacle_layer": obstacle,
              "inflation_layer": inflation}
    nodes = {
        "slam_toolbox": {
            "use_sim_time": False, "mode": "mapping",
            "odom_frame": of, "map_frame": mf, "base_frame": bf, "scan_topic": scan,
            "resolution": c["resolution"], "scan_queue_size": 1,
            "transform_publish_period": 0.05, "map_update_interval": 1.0,
            "minimum_time_interval": 0.1, "minimum_travel_distance": 0.1,
            "minimum_travel_heading": 0.1, "do_loop_closing": True,
            "solver_plugin": "solver_plugins::CeresSolver",
        },
        "map_server": {"use_sim_time": False, "yaml_filename": map_yaml,
                       "frame_id": mf, "topic_name": "map"},
        "amcl": {
            "use_sim_time": False, "base_frame_id": bf, "odom_frame_id": of,
            "global_frame_id": mf, "scan_topic": scan, "tf_broadcast": True,
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "min_particles": 500, "max_particles": 2000, "max_beams": 60,
            "laser_model_type": "likelihood_field", "transform_tolerance": 0.2,
            "update_min_d": 0.05, "update_min_a": 0.05, "set_initial_pose": False,
            "alpha1": 0.2, "alpha2": 0.2, "alpha3": 0.2, "alpha4": 0.2,
        },
        "planner_server": {
            "use_sim_time": False, "expected_planner_frequency": 1.0,
            "planner_plugins": ["GridBased"],
            "GridBased": {"plugin": "nav2_navfn_planner/NavfnPlanner",
                          "tolerance": c["xy_tolerance"], "use_astar": False,
                          "allow_unknown": False},
        },
        "controller_server": {
            "use_sim_time": False, "odom_topic": odom,
            "controller_frequency": c["controller_hz"],
            "min_x_velocity_threshold": 0.001, "min_y_velocity_threshold": 0.001,
            "min_theta_velocity_threshold": 0.001,
            "progress_checker_plugin": "progress_checker",
            "goal_checker_plugins": ["goal_checker"], "controller_plugins": ["FollowPath"],
            "progress_checker": {"plugin": "nav2_controller::SimpleProgressChecker",
                                 "required_movement_radius": 0.1,
                                 "movement_time_allowance": 20.0},
            "goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker",
                             "stateful": True, "xy_goal_tolerance": c["xy_tolerance"],
                             "yaw_goal_tolerance": c["yaw_tolerance"]},
            "FollowPath": {
                "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                "desired_linear_vel": c["max_speed"], "lookahead_dist": 0.4,
                "min_lookahead_dist": 0.2, "max_lookahead_dist": 0.6,
                "use_velocity_scaled_lookahead_dist": True, "lookahead_time": 1.5,
                "rotate_to_heading_angular_vel": c["max_yaw_speed"],
                "max_angular_accel": c["acceleration"],
                "rotate_to_heading_min_angle": 0.35,
                "use_rotate_to_heading": True, "allow_reversing": False,
                "use_collision_detection": True,
                "max_allowed_time_to_collision_up_to_carrot": 2.0,
                "use_regulated_linear_velocity_scaling": True,
                "use_cost_regulated_linear_velocity_scaling": True,
                "regulated_linear_scaling_min_speed": 0.03,
                "min_approach_linear_velocity": 0.02,
            },
        },
        "bt_navigator": {
            "use_sim_time": False, "global_frame": mf, "robot_base_frame": bf,
            "odom_topic": odom, "bt_loop_duration": 10,
            "default_server_timeout": 20,
        },
    }
    result = {name: {"ros__parameters": values} for name, values in nodes.items()}
    result["global_costmap"] = {"global_costmap": {"ros__parameters": {
        **common, "global_frame": mf, "update_frequency": 2.0, "publish_frequency": 1.0,
        "track_unknown_space": True, "rolling_window": False,
        "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
        "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer",
                         "map_subscribe_transient_local": True},
    }}}
    result["local_costmap"] = {"local_costmap": {"ros__parameters": {
        **common, "global_frame": of, "update_frequency": 10.0, "publish_frequency": 2.0,
        "rolling_window": True, "width": 5, "height": 5,
        "plugins": ["obstacle_layer", "inflation_layer"],
    }}}
    return result
