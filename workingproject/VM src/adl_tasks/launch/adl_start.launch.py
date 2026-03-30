# ---------- adl_start.launch.py ----------
# master launch file for ADL task nodes
# Starts: arm -> static scene obstacles -> vision node -> 
#           scene_from_vision -> ADL task node call (+UI for call interface)

# Usage:
# Defaults: [use_stub=true, launch_ui=true]
# ros2 launch adl_tasks adl_start.launch.py [use_stub:=true/false] [launch_ui:=true/false]

# import necessary ROS2 launch libraries
from launch import LaunchDescription                                        # define launch description
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable  # declare launch arguments
from launch.conditions import IfCondition, UnlessCondition                  # conditionally launch nodes
from launch.launch_description_sources import PythonLaunchDescriptionSource # include other launch files
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression  # access launch arguments
from launch_ros.actions import Node                                         # define ROS2 nodes
from launch_ros.substitutions import FindPackageShare                       # find package paths

# get default vision stub value from central config, if available
try:
    # central macro controls default vision mode
    from adl_tasks.adl_config import USE_VISION_STUB
    _DEFAULT_USE_STUB = "true" if bool(USE_VISION_STUB) else "false"
except Exception:
    _DEFAULT_USE_STUB = "true"

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
        default_value="192.168.0.1",
        description="Robot IP for Kinova launch.",
    )
    arm_fake_hw_arg = DeclareLaunchArgument(
        "arm_use_fake_hardware",
        default_value="true",
        description="Use fake hardware in Kinova launch. Set false for the physical arm.",
    )
    arm_use_joint_state_sanitizer_arg = DeclareLaunchArgument(
        "arm_use_joint_state_sanitizer",
        default_value="true",
        description=(
            "Use the ADL joint_state_sanitizer in the project arm launch. Disable for raw-state "
            "vendor-equivalent viewer debugging."
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
        description="Start the VBox USB camera publisher for the real AprilTag vision path.",
    )
    # - task/UI args - #
    launch_ui_arg = DeclareLaunchArgument(
        "launch_ui",
        default_value="true",
        description="Launch ADL UI node.",
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
                    "arm_vbox.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("arm_robot_ip"),
            "use_fake_hardware": LaunchConfiguration("arm_use_fake_hardware"),
            # [FLAG launch-arm-wrapper] Keep ADL/VBox-specific arm behavior in the project launch so
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
        condition=IfCondition(LaunchConfiguration("arm_use_fake_hardware")),
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
        # [FLAG launch-usb-camera-publisher] Start the VBox USB camera ROS bridge from the main
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
            use_stub_arg,
            start_usb_camera_publisher_arg,
            launch_ui_arg,
            #launch_tasks_arg,
            fake_hw_warning,
            moveit_topic_env_sanitized,
            moveit_topic_env_raw,
            arm_project_launch,
            scene_static_node,
            standalone_joint_state_sanitizer_node,
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
