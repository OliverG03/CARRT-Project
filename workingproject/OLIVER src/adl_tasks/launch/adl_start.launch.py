# ---------- adl_start.launch.py ----------
# master launch file for ADL task nodes
# Starts: arm -> static scene obstacles -> vision node -> 
#           scene_from_vision -> ADL task node call (+UI for call interface)

# Usage:
# Defaults: [use_stub=true, launch_ui=true]
# ros2 launch adl_tasks adl_start.launch.py [use_stub:=true/false] [launch_ui:=true/false]

# import necessary ROS2 launch libraries
from launch import LaunchDescription                                        # define launch description
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # declare launch arguments
from launch.conditions import IfCondition, UnlessCondition                  # conditionally launch nodes
from launch.launch_description_sources import PythonLaunchDescriptionSource # include other launch files
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # access launch arguments
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
        description="Use fake hardware in Kinova launch.",
    )
    # - vision args - #
    use_stub_arg = DeclareLaunchArgument(
        "use_stub",
        default_value=_DEFAULT_USE_STUB,
        description="Vision source selector: true=vision_stub, false=vision_apriltag.",
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
    kinova_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
                    "launch",
                    "robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("arm_robot_ip"),
            "use_fake_hardware": LaunchConfiguration("arm_use_fake_hardware"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_arm")),
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
    joint_state_sanitizer_node = Node(
        package="adl_tasks",
        executable="joint_state_sanitizer",
        name="joint_state_sanitizer_node",
        output="screen",
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
            use_stub_arg,
            launch_ui_arg,
            #launch_tasks_arg,
            kinova_launch,
            scene_static_node,
            joint_state_sanitizer_node,
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
