"""
Launch file for simulating a fixed-wing UAV with the gradient-based trajectory planner.

Based on: "Gradient-based trajectory planner of real-time obstacle avoidance for
fixed-wing unmanned aerial vehicles in cluttered environments."

Key differences from the quadrotor single_run_in_sim.launch.py:
  - use_fixed_wing = True (enables fixed-wing bound cost + curvature cost)
  - Higher max_vel / max_acc to reflect fixed-wing performance envelope
  - Larger planning_horizon (fixed-wing has higher cruise speed)
  - Wider map with higher z to allow altitude variation
  - Fixed-wing specific physical parameters in optimization namespace
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # ---- tunable arguments ----
    obj_num   = LaunchConfiguration('obj_num',   default=10)
    drone_id  = LaunchConfiguration('drone_id',  default=0)

    map_size_x = LaunchConfiguration('map_size_x', default=100.0)
    map_size_y = LaunchConfiguration('map_size_y', default=50.0)
    map_size_z = LaunchConfiguration('map_size_z', default=15.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    # Fixed-wing physical parameters (can be overridden at launch time)
    fw_mass       = LaunchConfiguration('fw_mass',       default=1.5)
    fw_wing_area  = LaunchConfiguration('fw_wing_area',  default=0.35)
    fw_C_D0       = LaunchConfiguration('fw_C_D0',       default=0.02)
    fw_k0         = LaunchConfiguration('fw_k0',         default=0.05)
    fw_rho        = LaunchConfiguration('fw_rho',        default=1.225)
    fw_T_max      = LaunchConfiguration('fw_T_max',      default=30.0)
    fw_T_min      = LaunchConfiguration('fw_T_min',      default=2.0)
    fw_n_max      = LaunchConfiguration('fw_n_max',      default=3.0)
    min_vel       = LaunchConfiguration('min_vel',        default=8.0)   # stall speed [m/s]
    max_curvature = LaunchConfiguration('max_curvature',  default=0.15)  # 1/min_turn_radius [1/m]

    # ---- declare arguments ----
    obj_num_cmd  = DeclareLaunchArgument('obj_num',  default_value=obj_num,  description='Number of obstacle objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)

    fw_mass_cmd       = DeclareLaunchArgument('fw_mass',       default_value=fw_mass,       description='Fixed-wing UAV mass [kg]')
    fw_wing_area_cmd  = DeclareLaunchArgument('fw_wing_area',  default_value=fw_wing_area,  description='Wing reference area [m^2]')
    fw_C_D0_cmd       = DeclareLaunchArgument('fw_C_D0',       default_value=fw_C_D0,       description='Zero-lift drag coefficient')
    fw_k0_cmd         = DeclareLaunchArgument('fw_k0',         default_value=fw_k0,         description='Lift-induced drag factor')
    fw_rho_cmd        = DeclareLaunchArgument('fw_rho',        default_value=fw_rho,        description='Air density [kg/m^3]')
    fw_T_max_cmd      = DeclareLaunchArgument('fw_T_max',      default_value=fw_T_max,      description='Maximum thrust [N]')
    fw_T_min_cmd      = DeclareLaunchArgument('fw_T_min',      default_value=fw_T_min,      description='Minimum thrust [N]')
    fw_n_max_cmd      = DeclareLaunchArgument('fw_n_max',      default_value=fw_n_max,      description='Maximum load factor')
    min_vel_cmd       = DeclareLaunchArgument('min_vel',        default_value=min_vel,       description='Minimum flight speed / stall speed [m/s]')
    max_curvature_cmd = DeclareLaunchArgument('max_curvature',  default_value=max_curvature, description='Maximum path curvature [1/m]')

    # ---- random forest obstacle map ----
    use_mockamap     = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap,
                                             description='Use mockamap instead of random_forest')

    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 80.0},
            {'map/y_size': 40.0},
            {'map/z_size': 12.0},
            {'map/resolution': 0.2},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 150},
            {'ObstacleShape/lower_rad': 1.0},
            {'ObstacleShape/upper_rad': 2.0},
            {'ObstacleShape/lower_hei': 2.0},
            {'ObstacleShape/upper_hei': 10.0},
            {'map/circle_num': 50},
            {'ObstacleShape/radius_l': 1.5},
            {'ObstacleShape/radius_h': 1.0},
            {'ObstacleShape/z_l': 2.0},
            {'ObstacleShape/z_h': 3.0},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 3.0},
        ],
        condition=UnlessCondition(use_mockamap),
    )

    # ---- ego_planner with fixed-wing settings ----
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id':     drone_id,
            'map_size_x_':  map_size_x,
            'map_size_y_':  map_size_y,
            'map_size_z_':  map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set':  obj_num,

            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic':       'pcl_render_node/depth',
            'cloud_topic':       'pcl_render_node/cloud',

            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),

            # Fixed-wing flight envelope
            'max_vel':          str(15.0),   # cruise speed  [m/s]
            'max_acc':          str(10.0),   # max accel     [m/s^2]
            'planning_horizon': str(20.0),   # longer horizon for fast UAV

            'use_distinctive_trajs': 'True',
            'flight_type': str(2),

            # Waypoints for a figure-eight style circuit
            'point_num': str(4),
            'point0_x': str(40.0), 'point0_y': str(0.0),  'point0_z': str(5.0),
            'point1_x': str(-40.0),'point1_y': str(0.0),  'point1_z': str(5.0),
            'point2_x': str(40.0), 'point2_y': str(0.0),  'point2_z': str(5.0),
            'point3_x': str(-40.0),'point3_y': str(0.0),  'point3_z': str(5.0),
            'point4_x': str(40.0), 'point4_y': str(0.0),  'point4_z': str(5.0),

            # Enable fixed-wing mode in the optimizer
            'use_fixed_wing': 'True',
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[
            ('position_cmd',     'planning/pos_cmd'),
            ('planning/bspline', 'planning/bspline'),
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},
        ],
    )

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic':   'False',
            'drone_id':      drone_id,
            'map_size_x_':   map_size_x,
            'map_size_y_':   map_size_y,
            'map_size_z_':   map_size_z,
            'init_x_':       str(-40.0),
            'init_y_':       str(0.0),
            'init_z_':       str(5.0),
            'odometry_topic': odom_topic,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_mockamap_cmd)

    ld.add_action(fw_mass_cmd)
    ld.add_action(fw_wing_area_cmd)
    ld.add_action(fw_C_D0_cmd)
    ld.add_action(fw_k0_cmd)
    ld.add_action(fw_rho_cmd)
    ld.add_action(fw_T_max_cmd)
    ld.add_action(fw_T_min_cmd)
    ld.add_action(fw_n_max_cmd)
    ld.add_action(min_vel_cmd)
    ld.add_action(max_curvature_cmd)

    ld.add_action(map_generator_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld
