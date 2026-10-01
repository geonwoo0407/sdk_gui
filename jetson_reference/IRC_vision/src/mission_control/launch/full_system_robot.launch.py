#!/usr/bin/env python3
"""Launch the STEP pipeline with an opt-in robot motion backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the hardware-capable graph with hardware disabled by default."""
    enable_camera = LaunchConfiguration("enable_camera")
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    show_camera_controls = LaunchConfiguration("show_camera_controls")
    metrics_mode = LaunchConfiguration("metrics_mode")
    max_fps = LaunchConfiguration("max_fps")
    max_rgb_depth_delta_sec = LaunchConfiguration(
        "max_rgb_depth_delta_sec"
    )
    overlay_max_stamp_delta_sec = LaunchConfiguration(
        "overlay_max_stamp_delta_sec"
    )

    camera_topic_prefix = LaunchConfiguration("camera_topic_prefix")

    color_image_topic = PythonExpression(
        ["'", camera_topic_prefix, "/color/image_raw'"]
    )
    aligned_depth_topic = PythonExpression(
        ["'", camera_topic_prefix, "/aligned_depth_to_color/image_raw'"]
    )
    color_camera_info_topic = PythonExpression(
        ["'", camera_topic_prefix, "/color/camera_info'"]
    )

    initial_mission_phase = LaunchConfiguration("initial_mission_phase")
    recovery_heading_turn_deg = LaunchConfiguration("recovery_heading_turn_deg")
    recovery_away_heading_turn_deg = LaunchConfiguration(
        "recovery_away_heading_turn_deg"
    )
    curve_follow_max_offset_norm = LaunchConfiguration(
        "curve_follow_max_offset_norm"
    )
    robot_center_offset_px = LaunchConfiguration("robot_center_offset_px")
    camera_height_m = LaunchConfiguration("camera_height_m")
    camera_pitch_down_deg = LaunchConfiguration("camera_pitch_down_deg")
    camera_forward_offset_m = LaunchConfiguration("camera_forward_offset_m")
    line_roi_x_min_ratio = LaunchConfiguration("line_roi_x_min_ratio")
    line_roi_x_max_ratio = LaunchConfiguration("line_roi_x_max_ratio")
    corner_min_turn_delta_deg = LaunchConfiguration("corner_min_turn_delta_deg")
    corner_straight_max_turn_delta_deg = LaunchConfiguration(
        "corner_straight_max_turn_delta_deg"
    )
    corner_straight_motion_distance_m = LaunchConfiguration(
        "corner_straight_motion_distance_m"
    )
    corner_turn_margin_m = LaunchConfiguration("corner_turn_margin_m")
    ball_tracking_range_m = LaunchConfiguration("ball_tracking_range_m")
    ball_control_range_m = LaunchConfiguration("ball_control_range_m")
    pickup_fine_step_bottom_distance_px = LaunchConfiguration(
        "pickup_fine_step_bottom_distance_px"
    )
    head_down_trigger_bottom_distance_px = LaunchConfiguration(
        "head_down_trigger_bottom_distance_px"
    )
    ball_head_override_deg = LaunchConfiguration("ball_head_override_deg")
    goal_tracking_range_m = LaunchConfiguration("goal_tracking_range_m")
    goal_control_range_m = LaunchConfiguration("goal_control_range_m")
    hurdle_control_range_m = LaunchConfiguration("hurdle_control_range_m")
    backend_type = LaunchConfiguration("backend_type")
    enable_robot_hardware = LaunchConfiguration("enable_robot_hardware")
    explicit_torque_approval = LaunchConfiguration(
        "explicit_torque_approval"
    )
    position_tolerance_enabled = LaunchConfiguration(
        "position_tolerance_enabled"
    )
    motion_json_path = LaunchConfiguration("motion_json_path")
    robot_device_path = LaunchConfiguration("robot_device_path")
    robot_baud_rate = LaunchConfiguration("robot_baud_rate")
    robot_motor_ids = LaunchConfiguration("robot_motor_ids")
    startup_pose_enabled = LaunchConfiguration("startup_pose_enabled")
    startup_pose_name = LaunchConfiguration("startup_pose_name")
    startup_pose_duration_ms = LaunchConfiguration("startup_pose_duration_ms")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "rgb_camera.color_profile": "1280,720,30",
            "depth_module.depth_profile": "848,480,30",
        }.items(),
    )

    detector = Node(
        package="step",
        executable="yolo26_detector",
        name="yolo26_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": model_path,
                "device": device,
                "display": ParameterValue(display, value_type=bool),
                "show_camera_controls": ParameterValue(
                    show_camera_controls,
                    value_type=bool,
                ),
                "metrics_mode": metrics_mode,
                "max_fps": ParameterValue(max_fps, value_type=float),
                "overlay_max_stamp_delta_sec": ParameterValue(
                    overlay_max_stamp_delta_sec,
                    value_type=float,
                ),
                "image_topic": ParameterValue(
                    color_image_topic,
                    value_type=str,
                ),
                "ball_tracking_range_m": ParameterValue(
                    ball_tracking_range_m, value_type=float
                ),
                "ball_control_range_m": ParameterValue(
                    ball_control_range_m, value_type=float
                ),
                "goal_tracking_range_m": ParameterValue(
                    goal_tracking_range_m, value_type=float
                ),
                "goal_control_range_m": ParameterValue(
                    goal_control_range_m, value_type=float
                ),
                "hurdle_tracking_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
                "hurdle_control_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
            }
        ],

    )

    unified_vision = Node(
        package="step",
        executable="unified_vision_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "image_topic": ParameterValue(
                    color_image_topic,
                    value_type=str,
                ),
                "depth_topic": ParameterValue(
                    aligned_depth_topic,
                    value_type=str,
                ),
                "camera_info_topic": ParameterValue(
                    color_camera_info_topic,
                    value_type=str,
                ),

                "robot_center_offset_px": ParameterValue(
                    robot_center_offset_px, value_type=float
                ),
                "camera_height_m": ParameterValue(
                    camera_height_m, value_type=float
                ),
                "camera_pitch_down_deg": ParameterValue(
                    camera_pitch_down_deg, value_type=float
                ),
                "camera_forward_offset_m": ParameterValue(
                    camera_forward_offset_m, value_type=float
                ),
                "max_rgb_depth_delta_sec": ParameterValue(
                    max_rgb_depth_delta_sec,
                    value_type=float,
                ),
                "roi_x_min_ratio": ParameterValue(
                    line_roi_x_min_ratio, value_type=float
                ),
                "roi_x_max_ratio": ParameterValue(
                    line_roi_x_max_ratio, value_type=float
                ),
                "corner_min_turn_delta_deg": ParameterValue(
                    corner_min_turn_delta_deg, value_type=float
                ),
                "corner_straight_max_turn_delta_deg": ParameterValue(
                    corner_straight_max_turn_delta_deg, value_type=float
                ),
                "corner_straight_motion_distance_m": ParameterValue(
                    corner_straight_motion_distance_m, value_type=float
                ),
                "corner_turn_margin_m": ParameterValue(
                    corner_turn_margin_m, value_type=float
                ),
                "head_down_trigger_bottom_distance_px": ParameterValue(
                    head_down_trigger_bottom_distance_px,
                    value_type=int,
                ),
            }
        ],

    )
    motion_decision = Node(
        package="mission_control",
        executable="motion_decision_node",
        name="motion_decision_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "initial_mission_phase": initial_mission_phase,
                "recovery_heading_turn_deg": ParameterValue(
                    recovery_heading_turn_deg, value_type=float
                ),
                "recovery_away_heading_turn_deg": ParameterValue(
                    recovery_away_heading_turn_deg, value_type=float
                ),
                "curve_follow_max_offset_norm": ParameterValue(
                    curve_follow_max_offset_norm, value_type=float
                ),
                "ball_tracking_range_m": ParameterValue(
                    ball_tracking_range_m, value_type=float
                ),
                "ball_control_range_m": ParameterValue(
                    ball_control_range_m, value_type=float
                ),
                "pickup_fine_step_bottom_distance_px": ParameterValue(
                    pickup_fine_step_bottom_distance_px,
                    value_type=int,
                ),
                "goal_tracking_range_m": ParameterValue(
                    goal_tracking_range_m, value_type=float
                ),
                "goal_control_range_m": ParameterValue(
                    goal_control_range_m, value_type=float
                ),
                "hurdle_control_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
            }
        ],
    )
    motion_command_bridge = Node(
        package="mission_control",
        executable="motion_command_bridge_node",
        name="motion_command_bridge_node",
        output="screen",
        emulate_tty=True,
    )
    sdk_motion_executor = Node(
        package="irc_step_motion_executor",
        executable="sdk_motion_executor",
        name="sdk_motion_executor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "backend_type": ParameterValue(backend_type, value_type=str),
                "enable_robot_hardware": ParameterValue(
                    enable_robot_hardware,
                    value_type=bool,
                ),
                "poll_period_ms": 5,
                "running_polls": 2,
                "settling_polls": 1,
                "explicit_torque_approval": ParameterValue(
                    explicit_torque_approval,
                    value_type=bool,
                ),
                "position_tolerance_enabled": ParameterValue(
                    position_tolerance_enabled,
                    value_type=bool,
                ),
                "motion_json_path": ParameterValue(
                    motion_json_path,
                    value_type=str,
                ),
                "robot_device_path": ParameterValue(
                    robot_device_path,
                    value_type=str,
                ),
                "robot_baud_rate": ParameterValue(
                    robot_baud_rate,
                    value_type=int,
                ),
                "robot_motor_ids": ParameterValue(robot_motor_ids),
                "startup_pose_enabled": ParameterValue(
                    startup_pose_enabled, value_type=bool
                ),
                "startup_pose_name": ParameterValue(
                    startup_pose_name, value_type=str
                ),
                "startup_pose_duration_ms": ParameterValue(
                    startup_pose_duration_ms, value_type=int
                ),
                "ball_head_override_deg": ParameterValue(
                    ball_head_override_deg,
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Camera is opt-in for robot startup safety.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("step"), "models", "best.engine"]
                ),
            ),
            DeclareLaunchArgument("device", default_value="tensorrt"),
            DeclareLaunchArgument("display", default_value="true"),
            DeclareLaunchArgument(
                "show_camera_controls",
                default_value="false",
            ),
            DeclareLaunchArgument("metrics_mode", default_value="auto"),
            DeclareLaunchArgument("max_fps", default_value="30.0"),
            DeclareLaunchArgument(
                "max_rgb_depth_delta_sec",
                default_value="0.05",
                description=(
                    "Maximum RGB/depth timestamp difference accepted for "
                    "ball control and metrics."
                ),
            ),
            DeclareLaunchArgument(
                "overlay_max_stamp_delta_sec",
                default_value="0.12",
                description=(
                    "Maximum RGB timestamp difference used only for the "
                    "detector debug overlay. This does not relax RGB-depth "
                    "control synchronization."
                ),
            ),
            DeclareLaunchArgument(
                "camera_topic_prefix",
                default_value=EnvironmentVariable(
                    "IRC_CAMERA_TOPIC_PREFIX",
                    default_value="/camera/camera",
                ),
                description=(
                    "RealSense topic prefix. "
                    "PC=/camera, Jetson=/camera/camera."
                ),
            ),
            DeclareLaunchArgument("initial_mission_phase", default_value="AUTO"),
            DeclareLaunchArgument("recovery_heading_turn_deg", default_value="10.0"),
            DeclareLaunchArgument(
                "recovery_away_heading_turn_deg", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "curve_follow_max_offset_norm", default_value="0.55"
            ),
            DeclareLaunchArgument("robot_center_offset_px", default_value="70.0"),
            DeclareLaunchArgument("camera_height_m", default_value="0.515"),
            DeclareLaunchArgument("camera_pitch_down_deg", default_value="45.0"),
            DeclareLaunchArgument("camera_forward_offset_m", default_value="0.0"),
            DeclareLaunchArgument("line_roi_x_min_ratio", default_value="0.15"),
            DeclareLaunchArgument("line_roi_x_max_ratio", default_value="0.85"),
            DeclareLaunchArgument(
                "corner_min_turn_delta_deg", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "corner_straight_max_turn_delta_deg", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "corner_straight_motion_distance_m", default_value="0.05"
            ),
            DeclareLaunchArgument("corner_turn_margin_m", default_value="0.15"),
            DeclareLaunchArgument("ball_tracking_range_m", default_value="1.5"),
            DeclareLaunchArgument("ball_control_range_m", default_value="1.5"),
            DeclareLaunchArgument(
                "pickup_fine_step_bottom_distance_px",
                default_value="500",
                description=(
                    "Switch to the fixed pickup fine step when the Ball "
                    "center is this many pixels or less from the image bottom."
                ),
            ),
            DeclareLaunchArgument(
                "head_down_trigger_bottom_distance_px",
                default_value="120",
            ),
            DeclareLaunchArgument(
                "ball_head_override_deg",
                default_value="-60.0",
            ),
            DeclareLaunchArgument("goal_tracking_range_m", default_value="1.0"),
            DeclareLaunchArgument("goal_control_range_m", default_value="0.5"),
            DeclareLaunchArgument("hurdle_control_range_m", default_value="1.0"),
            DeclareLaunchArgument(
                "backend_type",
                default_value="robot_motion_player",
                choices=["robot_motion_player"],
                description="The only supported backend for this robot launch.",
            ),
            DeclareLaunchArgument(
                "enable_robot_hardware",
                default_value="true",
                description="Must remain false until the hardware procedure is approved.",
            ),
            DeclareLaunchArgument(
                "explicit_torque_approval",
                default_value="true",
                description="Independent explicit approval for torque enable.",
            ),
            DeclareLaunchArgument(
                "position_tolerance_enabled",
                default_value="true",
                description=(
                    "Check final joint positions before completing each motion."
                ),
            ),
            DeclareLaunchArgument(
                "motion_json_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("irc_step_motion_executor"),
                        "config",
                        "robot_motions_runtime.json",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "robot_device_path", default_value="/dev/ttyUSB0"
            ),
            DeclareLaunchArgument("robot_baud_rate", default_value="4000000"),
            DeclareLaunchArgument(
                "robot_motor_ids",
                default_value=(
                    "[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,"
                    "18,19,20,21,22]"
                ),
                description="Integer array; empty by default and therefore unsafe to run.",
            ),
            DeclareLaunchArgument("startup_pose_enabled", default_value="true"),
            DeclareLaunchArgument("startup_pose_name", default_value="오뒤410"),
            DeclareLaunchArgument("startup_pose_duration_ms", default_value="4000"),
            camera,
            detector,
            unified_vision,
            motion_decision,
            motion_command_bridge,
            sdk_motion_executor,
        ]
    )
