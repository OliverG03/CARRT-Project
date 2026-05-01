# ---------- adl_start.launch.py ----------
# master launch file for ADL task nodes
# Starts: arm -> static scene obstacles -> vision node -> 
#           scene_from_vision -> ADL task node call (+UI for call interface)

# Usage:
# Defaults: [use_stub=false, launch_ui=true]
# ros2 launch adl_tasks adl_start.launch.py [use_stub:=true/false] [launch_ui:=true/false]

# import necessary ROS2 launch libraries
from launch import LaunchDescription                                        # define launch description
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable  # declare launch arguments
from launch.conditions import IfCondition, UnlessCondition                  # conditionally launch nodes
from launch.launch_description_sources import PythonLaunchDescriptionSource # include other launch files
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression  # access launch arguments
from launch_ros.actions import Node                                         # define ROS2 nodes
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare                       # find package paths

# get default vision stub value from central config, if available
try:
    # central macro controls default vision mode and shared measured scene defaults
    from adl_tasks.adl_config import (
        USE_VISION_STUB,
        LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M,
        SCENE_CALIBRATION_X_OFFSET_M,
        SCENE_CALIBRATION_Y_OFFSET_M,
        SCENE_CALIBRATION_Z_OFFSET_M,
        SCENE_CALIBRATION_YAW_DEG,
        SCENE_CALIBRATION_XY_SCALE,
        TABLE_CENTER_Y_FROM_BASE_M,
        TABLE_FRONT_EDGE_FROM_BASE_M,
    )
    _DEFAULT_USE_STUB = "true" if bool(USE_VISION_STUB) else "false"
    _DEFAULT_SCENE_CALIBRATION_X_OFFSET_M = f"{float(SCENE_CALIBRATION_X_OFFSET_M):.6f}"
    _DEFAULT_SCENE_CALIBRATION_Y_OFFSET_M = f"{float(SCENE_CALIBRATION_Y_OFFSET_M):.6f}"
    _DEFAULT_SCENE_CALIBRATION_Z_OFFSET_M = f"{float(SCENE_CALIBRATION_Z_OFFSET_M):.6f}"
    _DEFAULT_SCENE_CALIBRATION_YAW_DEG = f"{float(SCENE_CALIBRATION_YAW_DEG):.6f}"
    _DEFAULT_SCENE_CALIBRATION_XY_SCALE = f"{float(SCENE_CALIBRATION_XY_SCALE):.6f}"
    _DEFAULT_TABLE_FRONT_EDGE_FROM_BASE_M = f"{float(TABLE_FRONT_EDGE_FROM_BASE_M):.6f}"
    _DEFAULT_TABLE_CENTER_Y_FROM_BASE_M = f"{float(TABLE_CENTER_Y_FROM_BASE_M):.6f}"
    _DEFAULT_LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M = (
        f"{float(LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M):.6f}"
    )
except Exception:
    _DEFAULT_USE_STUB = "false"
    _DEFAULT_SCENE_CALIBRATION_X_OFFSET_M = "0.000000"
    _DEFAULT_SCENE_CALIBRATION_Y_OFFSET_M = "0.000000"
    _DEFAULT_SCENE_CALIBRATION_Z_OFFSET_M = "0.000000"
    _DEFAULT_SCENE_CALIBRATION_YAW_DEG = "0.000000"
    _DEFAULT_SCENE_CALIBRATION_XY_SCALE = "1.000000"
    _DEFAULT_TABLE_FRONT_EDGE_FROM_BASE_M = "0.406400"
    _DEFAULT_TABLE_CENTER_Y_FROM_BASE_M = "0.000000"
    _DEFAULT_LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M = f"{9.5 * 0.0254:.6f}"


def _wrist_camera_static_tf_nodes(condition):
    # [FLAG launch-standalone-wrist-camera-tf] When start_arm:=false and the vendor arm launch runs
    # in a separate terminal, bridge vendor camera_color_frame to the frame stamped by your USB
    # camera publisher. On the real arm, camera_color_frame is already aligned with the USB/AprilTag
    # optical image axes for the table scan pose, so do not apply a second optical-frame rotation.
    camera_tf_chain = [
        (
            "camera_color_optical_bridge_tf_pub",
            "camera_color_frame",
            "wrist_mounted_camera_color_optical_frame",
            ("0.0", "0.0", "0.0"),
            ("0.0", "0.0", "0.0"),
        ),
    ]

    static_tf_nodes = []
    for node_name, parent_frame, child_frame, xyz, rpy in camera_tf_chain:
        static_tf_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=node_name,
                output="log",
                arguments=[
                    "--x", xyz[0],
                    "--y", xyz[1],
                    "--z", xyz[2],
                    "--roll", rpy[0],
                    "--pitch", rpy[1],
                    "--yaw", rpy[2],
                    "--frame-id", parent_frame,
                    "--child-frame-id", child_frame,
                ],
                condition=condition,
            )
        )
    return static_tf_nodes

# main launch description generation function
def generate_launch_description() -> LaunchDescription:
    # --- Launch arguments --- #
    
    # - arm bringup args - #
    start_arm_arg = DeclareLaunchArgument(
        "start_arm",
        default_value="true",
        description="Start Kinova arm MoveIt/control launch in this launch file.",
    )
    arm_robot_ip_arg = DeclareLaunchArgument(
        "arm_robot_ip",
        default_value="192.168.0.10",
        description="Robot IP for Kinova launch. Defaults to the lab Gen3 controller at 192.168.0.10.",
    )
    arm_fake_hw_arg = DeclareLaunchArgument(
        "arm_use_fake_hardware",
        default_value="true",
        description=(
            "Use fake hardware in Kinova launch. Set false for the physical arm. When "
            "start_arm:=false and use_stub:=false, the ADL task nodes assume an externally "
            "launched real arm by default."
        ),
    )
    arm_use_joint_state_sanitizer_arg = DeclareLaunchArgument(
        "arm_use_joint_state_sanitizer",
        default_value="true",
        description=(
            "Use the ADL joint_state_sanitizer in the project arm launch. Disable for raw-state "
            "vendor-equivalent viewer debugging."
        ),
    )
    arm_short_cartesian_mode_arg = DeclareLaunchArgument(
        "arm_short_cartesian_mode",
        default_value="auto",
        description=(
            "Short Cartesian backend for ADL tasks. auto enables the hybrid twist-controller "
            "path on real hardware, including the split bringup case where the arm is launched "
            "in another terminal. planned forces MoveIt-only short motions."
        ),
    )
    # - vision args - #
    use_stub_arg = DeclareLaunchArgument(
        "use_stub",
        default_value=_DEFAULT_USE_STUB,
        description="Vision source selector: true=vision_stub, false=vision_apriltag.",
    )
    start_usb_camera_publisher_arg = DeclareLaunchArgument(
        "start_usb_camera_publisher",
        default_value="true",
        description="Start the USB camera publisher for the real AprilTag vision path.",
    )
    scene_memory_mode_arg = DeclareLaunchArgument(
        "scene_memory_mode",
        default_value="true",
        description="Keep remembered object poses when tags are briefly lost.",
    )
    scene_continuous_updates_arg = DeclareLaunchArgument(
        "scene_continuous_scene_updates",
        default_value="false",
        description="Continuously refresh scene object poses while tags remain visible.",
    )
    scene_latch_first_detection_arg = DeclareLaunchArgument(
        "scene_latch_first_detection_updates",
        default_value="true",
        description="Latch the first visible object pose into memory when continuous updates are off.",
    )
    scene_scan_prune_unseen_arg = DeclareLaunchArgument(
        "scene_scan_prune_unseen_objects",
        default_value="false",
        description="During scan_scene calibration, remove remembered objects not reacquired in the scan.",
    )
    scene_log_object_pose_debug_arg = DeclareLaunchArgument(
        "scene_log_object_pose_debug",
        default_value="false",
        description="Log extra tag-to-object geometry details for cup/medication calibration.",
    )
    scene_calibration_x_offset_arg = DeclareLaunchArgument(
        "scene_calibration_x_offset_m",
        default_value=_DEFAULT_SCENE_CALIBRATION_X_OFFSET_M,
        description="Global scene calibration X offset in meters.",
    )
    scene_calibration_y_offset_arg = DeclareLaunchArgument(
        "scene_calibration_y_offset_m",
        default_value=_DEFAULT_SCENE_CALIBRATION_Y_OFFSET_M,
        description="Global scene calibration Y offset in meters.",
    )
    scene_calibration_z_offset_arg = DeclareLaunchArgument(
        "scene_calibration_z_offset_m",
        default_value=_DEFAULT_SCENE_CALIBRATION_Z_OFFSET_M,
        description="Global scene calibration Z offset in meters.",
    )
    scene_calibration_yaw_arg = DeclareLaunchArgument(
        "scene_calibration_yaw_deg",
        default_value=_DEFAULT_SCENE_CALIBRATION_YAW_DEG,
        description="Global scene calibration yaw in degrees.",
    )
    scene_calibration_xy_scale_arg = DeclareLaunchArgument(
        "scene_calibration_xy_scale",
        default_value=_DEFAULT_SCENE_CALIBRATION_XY_SCALE,
        description="Global scene calibration XY scale.",
    )
    table_front_edge_from_base_arg = DeclareLaunchArgument(
        "scene_table_front_edge_from_base_m",
        default_value=_DEFAULT_TABLE_FRONT_EDGE_FROM_BASE_M,
        description="Measured distance from the arm base center to the real front table edge in meters.",
    )
    table_center_y_from_base_arg = DeclareLaunchArgument(
        "scene_table_center_y_from_base_m",
        default_value=_DEFAULT_TABLE_CENTER_Y_FROM_BASE_M,
        description="Measured signed Y offset from the arm base center to the real table center in meters.",
    )
    left_desk_wall_clearance_arg = DeclareLaunchArgument(
        "scene_left_desk_wall_clearance_from_table_edge_m",
        default_value=_DEFAULT_LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M,
        description="Measured clearance from the table left edge to the desk wall near face in meters.",
    )
    # - task/UI args - #
    launch_ui_arg = DeclareLaunchArgument(
        "launch_ui",
        default_value="true",
        description="Launch ADL UI node.",
    )
    clear_table_top_only_scan_arg = DeclareLaunchArgument(
        "clear_table_top_only_scan",
        default_value="false",
        description=(
            "Legacy diagnostic flag. Deterministic clear_table scans still run side sweeps "
            "when side-grasp target IDs are configured."
        ),
    )
    clear_table_require_side_sweep_arg = DeclareLaunchArgument(
        "clear_table_require_side_sweep",
        default_value="true",
        description=(
            "Diagnostic compatibility flag. Deterministic clear_table scans now force side sweeps "
            "whenever side-grasp target IDs are configured."
        ),
    )
    clear_table_cup_alignment_enable_arg = DeclareLaunchArgument(
        "clear_table_cup_alignment_enable",
        default_value="true",
        description=(
            "Enable the cup's Stage 1 live QR alignment verification in clear_table. "
            "Set false to skip the cup reread/hard-gate and continue from the settled side lane."
        ),
    )
    '''
    launch_tasks_arg = DeclareLaunchArgument(
        "launch_tasks",
        default_value="true",
        description="Launch ADL task nodes.",
    )
    '''
    # --- arm bringup --- #
    arm_project_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("adl_tasks"),
                    "launch",
                    "arm_start.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("arm_robot_ip"),
            "use_fake_hardware": LaunchConfiguration("arm_use_fake_hardware"),
            # [FLAG launch-arm-wrapper] Keep ADL-specific arm behavior in the project launch so
            # vendor robot.launch.py can remain the baseline reference bringup.
            "use_joint_state_sanitizer": LaunchConfiguration("arm_use_joint_state_sanitizer"),
            # [FLAG launch-project-vision-frames] ADL vision tasks need the wrist camera frames in
            # the robot description, so the project arm launch keeps vision enabled by default.
            "vision": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_arm")),
    )
    fake_hw_warning = LogInfo(
        msg=(
            "[adl_start.launch] arm_use_fake_hardware:=true, so RViz/MoveIt will show fake "
            "arm state instead of the physical Kinova."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_arm"),
                    "' == 'true' and '",
                    LaunchConfiguration("arm_use_fake_hardware"),
                    "' == 'true'",
                ]
            )
        ),
    )
    split_real_arm_inference_info = LogInfo(
        msg=(
            "[adl_start.launch] start_arm:=false with live AprilTag vision detected. "
            "Assuming the Kinova is running in another terminal, so ADL task nodes will enable "
            "real-hardware-only short Cartesian helpers."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_arm"),
                    "' == 'false' and '",
                    LaunchConfiguration("use_stub"),
                    "' == 'false' and '",
                    LaunchConfiguration("arm_use_fake_hardware"),
                    "' == 'true'",
                ]
            )
        ),
    )
    moveit_topic_env_sanitized = SetEnvironmentVariable(
        name="ADL_MOVEIT_JOINT_STATES_TOPIC",
        value="/joint_states_sanitized",
        condition=IfCondition(LaunchConfiguration("arm_use_joint_state_sanitizer")),
    )
    moveit_topic_env_raw = SetEnvironmentVariable(
        name="ADL_MOVEIT_JOINT_STATES_TOPIC",
        value="/joint_states",
        condition=UnlessCondition(LaunchConfiguration("arm_use_joint_state_sanitizer")),
    )
    real_hardware_mode_env = SetEnvironmentVariable(
        name="ADL_REAL_HARDWARE_MODE",
        value=PythonExpression(
            [
                "'true' if ('",
                LaunchConfiguration("arm_use_fake_hardware"),
                "' == 'false' or ('",
                LaunchConfiguration("start_arm"),
                "' == 'false' and '",
                LaunchConfiguration("use_stub"),
                "' == 'false')) else 'false'",
            ]
        ),
    )
    short_cartesian_mode_env = SetEnvironmentVariable(
        name="ADL_SHORT_CARTESIAN_MODE",
        value=LaunchConfiguration("arm_short_cartesian_mode"),
    )
    table_front_edge_env = SetEnvironmentVariable(
        name="ADL_TABLE_FRONT_EDGE_FROM_BASE_M",
        value=LaunchConfiguration("scene_table_front_edge_from_base_m"),
    )
    table_center_y_env = SetEnvironmentVariable(
        name="ADL_TABLE_CENTER_Y_FROM_BASE_M",
        value=LaunchConfiguration("scene_table_center_y_from_base_m"),
    )
    left_desk_wall_clearance_env = SetEnvironmentVariable(
        name="ADL_LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M",
        value=LaunchConfiguration("scene_left_desk_wall_clearance_from_table_edge_m"),
    )
    clear_table_top_only_scan_env = SetEnvironmentVariable(
        name="ADL_CLEAR_TABLE_TOP_ONLY_SCAN",
        value=LaunchConfiguration("clear_table_top_only_scan"),
    )
    clear_table_require_side_sweep_env = SetEnvironmentVariable(
        name="ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP",
        value=LaunchConfiguration("clear_table_require_side_sweep"),
    )
    clear_table_cup_alignment_enable_env = SetEnvironmentVariable(
        name="ADL_CLEAR_TABLE_CUP_ALIGNMENT_ENABLE",
        value=LaunchConfiguration("clear_table_cup_alignment_enable"),
    )
    standalone_joint_state_sanitizer_node = Node(
        package="adl_tasks",
        executable="joint_state_sanitizer",
        name="joint_state_sanitizer_node",
        output="screen",
        # [FLAG launch-standalone-sanitizer] Support the safer split bringup where the vendor arm
        # launch runs in one terminal and the ADL stack runs with start_arm:=false in another.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_arm"),
                    "' == 'false' and '",
                    LaunchConfiguration("arm_use_joint_state_sanitizer"),
                    "' == 'true'",
                ]
            )
        ),
    )
    standalone_wrist_camera_tf_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_arm"),
                "' == 'false' and '",
                LaunchConfiguration("use_stub"),
                "' == 'false'",
            ]
        )
    )
    standalone_wrist_camera_tf_nodes = _wrist_camera_static_tf_nodes(
        condition=standalone_wrist_camera_tf_condition
    )
    # --- scene + vision --- #
    scene_static_node = Node(
        package="adl_tasks",
        executable="scene_static",
        name="scene_static_node",
        output="screen",
    )
    vision_stub_node = Node(
        package="adl_tasks",
        executable="vision_stub",
        name="vision_stub_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_stub")),
    )
    vision_apriltag_node = Node(
        package="adl_tasks",
        executable="vision_apriltag",
        name="vision_apriltag_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_stub")),
    )
    usb_camera_publisher_node = Node(
        package="adl_tasks",
        executable="wrist_camera_usb_publisher",
        name="wrist_camera_usb_publisher_node",
        output="screen",
        # [FLAG launch-usb-camera-publisher] Start the USB camera ROS bridge from the main
        # ADL launch when using real vision so /wrist_mounted_camera/image exists without an
        # extra manual terminal. It stays disable-able for deployments with a true ROS camera driver.
        # [FLAG launch-bool-quoting] PythonExpression evals launch substitutions as raw text, so
        # unquoted true/false become invalid Python names. Wrap each substitution in quotes so the
        # condition compares string values reliably at launch time.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_usb_camera_publisher"),
                    "' == 'true' and '",
                    LaunchConfiguration("use_stub"),
                    "' == 'false'",
                ]
            )
        ),
    )
    scene_from_vision_node = Node(
        package="adl_tasks",
        executable="scene_from_vision",
        name="scene_from_vision_node",
        output="screen",
        parameters=[
            {
                "memory_mode": ParameterValue(
                    LaunchConfiguration("scene_memory_mode"),
                    value_type=bool,
                ),
                "continuous_scene_updates": ParameterValue(
                    LaunchConfiguration("scene_continuous_scene_updates"),
                    value_type=bool,
                ),
                "latch_first_detection_updates": ParameterValue(
                    LaunchConfiguration("scene_latch_first_detection_updates"),
                    value_type=bool,
                ),
                "scan_prune_unseen_objects": ParameterValue(
                    LaunchConfiguration("scene_scan_prune_unseen_objects"),
                    value_type=bool,
                ),
                "log_object_pose_debug": ParameterValue(
                    LaunchConfiguration("scene_log_object_pose_debug"),
                    value_type=bool,
                ),
                "scene_calibration_x_offset_m": ParameterValue(
                    LaunchConfiguration("scene_calibration_x_offset_m"),
                    value_type=float,
                ),
                "scene_calibration_y_offset_m": ParameterValue(
                    LaunchConfiguration("scene_calibration_y_offset_m"),
                    value_type=float,
                ),
                "scene_calibration_z_offset_m": ParameterValue(
                    LaunchConfiguration("scene_calibration_z_offset_m"),
                    value_type=float,
                ),
                "scene_calibration_yaw_deg": ParameterValue(
                    LaunchConfiguration("scene_calibration_yaw_deg"),
                    value_type=float,
                ),
                "scene_calibration_xy_scale": ParameterValue(
                    LaunchConfiguration("scene_calibration_xy_scale"),
                    value_type=float,
                ),
            }
        ],
    )
    # --- tasks + UI --- #
    
    clear_table_node = Node(
        package="adl_tasks",
        executable="clear_table",
        name="clear_table_node",
        output="screen",
        #condition=IfCondition(LaunchConfiguration("launch_tasks")),
    )
    pick_bottle_node = Node(
        package="adl_tasks",
        executable="pick_dropped_bottle",
        name="pick_dropped_bottle_node",
        output="screen",
        # [FLAG launch-all-tasks] load all ADL task nodes so UI commands always have subscribers.
    )
    give_medication_node = Node(
        package="adl_tasks",
        executable="give_medication",
        name="give_medication_node",
        output="screen",
        # [FLAG launch-all-tasks] load all ADL task nodes so UI commands always have subscribers.
    )
    controller_node = Node(
        package="adl_tasks",
        executable="adl_controller",
        name="adl_controller_node",
        output="screen",
        # [FLAG launch-shared-controller] Own startup/idle parking plus emergency-stop and turn-off policy
        # in one node so multiple task processes do not race to command the same arm.
    )
    ui_node = Node(
        package="adl_tasks",
        executable="adl_ui",
        name="adl_ui_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_ui")),
    )
    ui_node = Node(
        package="adl_tasks",
        executable="adl_ui",
        name="adl_ui_node",
        output="screen",
        # [FLAG launch-ui-toggle]: keep UI optional for headless tests.
        condition=IfCondition(LaunchConfiguration("launch_ui")),
    )
    # return the full launch description with all nodes and arguments
    # allows for modular launching of components based on arguments (e.g. use_stub, launch_ui, launch_tasks)
    return LaunchDescription(
        [
            start_arm_arg,
            arm_robot_ip_arg,
            arm_fake_hw_arg,
            arm_use_joint_state_sanitizer_arg,
            arm_short_cartesian_mode_arg,
            use_stub_arg,
            start_usb_camera_publisher_arg,
            scene_memory_mode_arg,
            scene_continuous_updates_arg,
            scene_latch_first_detection_arg,
            scene_scan_prune_unseen_arg,
            scene_log_object_pose_debug_arg,
            scene_calibration_x_offset_arg,
            scene_calibration_y_offset_arg,
            scene_calibration_z_offset_arg,
            scene_calibration_yaw_arg,
            scene_calibration_xy_scale_arg,
            table_front_edge_from_base_arg,
            table_center_y_from_base_arg,
            left_desk_wall_clearance_arg,
            launch_ui_arg,
            clear_table_top_only_scan_arg,
            clear_table_require_side_sweep_arg,
            clear_table_cup_alignment_enable_arg,
            #launch_tasks_arg,
            fake_hw_warning,
            split_real_arm_inference_info,
            moveit_topic_env_sanitized,
            moveit_topic_env_raw,
            real_hardware_mode_env,
            short_cartesian_mode_env,
            table_front_edge_env,
            table_center_y_env,
            left_desk_wall_clearance_env,
            clear_table_top_only_scan_env,
            clear_table_require_side_sweep_env,
            clear_table_cup_alignment_enable_env,
            arm_project_launch,
            scene_static_node,
            standalone_joint_state_sanitizer_node,
            *standalone_wrist_camera_tf_nodes,
            usb_camera_publisher_node,
            vision_stub_node,
            vision_apriltag_node,
            scene_from_vision_node,
            controller_node,
            pick_bottle_node,
            clear_table_node,
            give_medication_node,
            ui_node,
        ]
    )
