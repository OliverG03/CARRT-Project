import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _parse_cpu_affinity(value: str):
    raw = str(value).strip()
    if not raw:
        return None
    cores = [int(part.strip()) for part in raw.split(",") if part.strip()]
    if not cores:
        return None
    return cores[0] if len(cores) == 1 else cores


def _wrist_camera_static_tf_nodes(condition):
    # [FLAG launch-wrist-camera-static-tf] Keep the frame-name bridge in project launch space
    # instead of patching vendor Kortex Xacro. On the real arm, camera_color_frame is already
    # aligned with the USB/AprilTag optical image axes for the table scan pose, so do not apply
    # a second optical-frame rotation here.
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


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_joint_state_sanitizer = LaunchConfiguration("use_joint_state_sanitizer")
    motion_only_mode = LaunchConfiguration("motion_only_mode")
    controller_update_rate = LaunchConfiguration("controller_update_rate")
    vision = LaunchConfiguration("vision")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    lock_memory = LaunchConfiguration("lock_memory")
    controller_thread_priority = LaunchConfiguration("controller_thread_priority")
    controller_cpu_affinity = LaunchConfiguration("controller_cpu_affinity")
    controller_print_overrun_warnings = LaunchConfiguration("controller_print_overrun_warnings")
    adl_share_dir = get_package_share_directory("adl_tasks")
    use_sanitizer = _as_bool(use_joint_state_sanitizer.perform(context))
    motion_only = _as_bool(motion_only_mode.perform(context))
    use_fake_hw = _as_bool(use_fake_hardware.perform(context))
    configured_update_rate = controller_update_rate.perform(context).strip()
    explicit_update_rate_override = bool(configured_update_rate)
    configured_cpu_affinity = _parse_cpu_affinity(controller_cpu_affinity.perform(context))
    if not configured_update_rate:
        # [FLAG arm-real-hardware-rate] Keep the project bringup conservative on the physical arm.
        # The recent startup logs are repeatedly missing 1000 Hz badly enough to trigger immediate
        # path-tolerance aborts before vision even starts, so default the real arm to the slower
        # ADL profile unless the user explicitly asks for a different rate.
        if use_fake_hw:
            configured_update_rate = "50" if motion_only else "125"
        else:
            configured_update_rate = "50"
    if not use_fake_hw:
        try:
            requested_update_rate = int(configured_update_rate)
        except Exception:
            requested_update_rate = 50
        if requested_update_rate > 125 and not explicit_update_rate_override:
            print(
                "[arm_start.launch] controller_update_rate above 125 Hz on real hardware has been "
                "causing repeated startup/control aborts; clamping to 50 Hz for this launch."
            )
            configured_update_rate = "50"
        elif requested_update_rate > 125 and explicit_update_rate_override:
            print(
                "[arm_start.launch] controller_update_rate was explicitly overridden above 125 Hz "
                f"for real hardware ({requested_update_rate} Hz); honoring the override for "
                "operator tuning even though recent logs showed overruns at that rate."
            )

    # [FLAG arm-project-wrapper] Keep project-specific planning state handling in the ADL package
    # instead of patching the vendor Kortex launch. This wrapper mirrors the vendor bringup
    # structure, but it is only for the project build path; direct vendor robot.launch.py remains
    # the baseline arm launch for stock debugging.
    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "vision": vision,
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_config.moveit_cpp.update({"use_sim_time": _as_bool(use_sim_time.perform(context))})
    move_group_remappings = []
    if use_sanitizer:
        # [FLAG arm-vbox-sanitized-moveit] ADL helpers expect MoveIt to consume the sanitized arm
        # state stream so wrapped joints do not poison start-state validation in the VM.
        move_group_remappings.append(("/joint_states", "/joint_states_sanitized"))
    move_group_joint_state_topic = "/joint_states_sanitized" if use_sanitizer else "/joint_states"
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": move_group_joint_state_topic,
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        # Mirror the nested parameter with a flat override as well. The MoveIt builder can inject
        # its own default monitor options, and this keeps the final parameter merge deterministic.
        "planning_scene_monitor_options.joint_state_topic": move_group_joint_state_topic,
    }
    move_group_parameters = [
        moveit_config.to_dict(),
        planning_scene_monitor_parameters,
    ]
    if motion_only:
        # [FLAG arm-motion-only-moveit] The Kinova API examples keep arm motion and gripper
        # control as separate operations. Mirror that in the arm-only test path so MoveIt does
        # not waste time discovering or monitoring a gripper controller that is intentionally absent.
        move_group_parameters.append(
            {
                "moveit_controller_manager": (
                    "moveit_simple_controller_manager/MoveItSimpleControllerManager"
                ),
                "moveit_simple_controller_manager": {
                    "controller_names": ["joint_trajectory_controller"],
                    "joint_trajectory_controller": {
                        "type": "FollowJointTrajectory",
                        "action_ns": "follow_joint_trajectory",
                        "default": True,
                        "joints": [
                            "joint_1",
                            "joint_2",
                            "joint_3",
                            "joint_4",
                            "joint_5",
                            "joint_6",
                            "joint_7",
                        ],
                    },
                },
            }
        )
    # [FLAG arm-quiet-console] Motion-only tests are already dominated by controller write
    # stalls. Keep high-volume node logs in rosout/log files instead of flooding the interactive
    # terminal during real-arm motion tests.
    motion_test_output = "log" if motion_only else "screen"
    motion_test_output_both = "log" if motion_only else "both"

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output=motion_test_output,
        parameters=move_group_parameters,
        remappings=move_group_remappings,
    )
    move_group_state_guard = None
    if use_sanitizer:
        move_group_state_guard_triggered = {"value": False}

        def _guard_move_group_state_source(event):
            if move_group_state_guard_triggered["value"]:
                return None
            text = event.text.decode(errors="ignore").strip()
            if not text:
                return None

            reason = None
            if "Found empty JointState message" in text:
                reason = (
                    "move_group reported an empty JointState while sanitizer mode is active. "
                    "Keeping the launch alive, but this still indicates a planning-state problem."
                )
            if reason is None:
                return None

            move_group_state_guard_triggered["value"] = True
            return [LogInfo(msg=f"[arm_start.launch] {reason} move_group output: {text}")]

        move_group_state_guard = RegisterEventHandler(
            event_handler=OnProcessIO(
                target_action=move_group_node,
                on_stdout=_guard_move_group_state_source,
                on_stderr=_guard_move_group_state_source,
            )
        )

    joint_state_sanitizer_node = Node(
        package="adl_tasks",
        executable="joint_state_sanitizer",
        name="joint_state_sanitizer_node",
        output=motion_test_output,
        condition=IfCondition(use_joint_state_sanitizer),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output=motion_test_output_both,
        parameters=[
            moveit_config.robot_description,
        ],
    )

    if motion_only:
        # [FLAG arm-motion-profile] For arm-only motion debugging, keep only the arm trajectory
        # and joint-state controllers alive. This removes gripper/fault/twist controller switching
        # from the startup path so the control loop has a smaller realtime burden.
        ros2_controllers_filename = "ros2_controllers_vbox_motion.yaml"
    else:
        ros2_controllers_filename = "ros2_controllers_vbox.yaml"
    ros2_controllers_path = os.path.join(adl_share_dir, "config", ros2_controllers_filename)
    ros2_control_parameters = [
        ros2_controllers_path,
        {
            # [FLAG arm-rate-override] Inline launch dictionaries target the node's
            # parameter namespace directly, unlike YAML files which wrap values under
            # controller_manager.ros__parameters. Keep this flat so controller_update_rate:=N
            # actually overrides the installed YAML instead of silently leaving 500 Hz active.
            "update_rate": int(configured_update_rate),
            # [FLAG arm-memory-lock] ros2_control supports locking memory to cut page-fault
            # jitter. Leave this configurable from the project wrapper so launch tuning stays in
            # adl_tasks instead of leaking into vendor launch files.
            "lock_memory": _as_bool(lock_memory.perform(context)),
            # [FLAG arm-thread-priority] Expose controller_manager priority from launch so the
            # motion wrapper can experiment with scheduler settings without editing vendor files.
            "thread_priority": int(controller_thread_priority.perform(context)),
            # [FLAG arm-overrun-print] The overrun warnings are diagnostically useful, but the
            # warning flood can add extra console/log pressure. Keep it configurable.
            "overruns.print_warnings": _as_bool(
                controller_print_overrun_warnings.perform(context)
            ),
        },
    ]
    if configured_cpu_affinity is not None:
        # [FLAG arm-cpu-affinity] Pinning controller_manager to a stable CPU can reduce
        # scheduler jitter on overloaded hosts. Accept either a single core like "2" or a list
        # like "2,3" from launch.
        ros2_control_parameters[1]["cpu_affinity"] = configured_cpu_affinity
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=ros2_control_parameters,
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output=motion_test_output_both,
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
        condition=UnlessCondition(motion_only_mode),
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(motion_only_mode),
    )

    fault_controller_spawner = None
    if not motion_only:
        fault_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["fault_controller", "-c", "/controller_manager"],
            condition=UnlessCondition(use_fake_hardware),
        )

    rviz_config_file = (
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    if use_sanitizer:
        state_source_msg = (
            "[arm_start.launch] MoveIt current state source: /joint_states_sanitized. "
            "Use use_joint_state_sanitizer:=false for raw-state viewer debugging."
        )
    else:
        state_source_msg = (
            "[arm_start.launch] MoveIt current state source: raw /joint_states. "
            "ADL task helpers expect /joint_states_sanitized when motion tasks are enabled."
        )
    state_source_info = LogInfo(msg=state_source_msg)
    moveit_topic_env_sanitized = SetEnvironmentVariable(
        name="ADL_MOVEIT_JOINT_STATES_TOPIC",
        value="/joint_states_sanitized",
        condition=IfCondition(use_joint_state_sanitizer),
    )
    moveit_topic_env_raw = SetEnvironmentVariable(
        name="ADL_MOVEIT_JOINT_STATES_TOPIC",
        value="/joint_states",
        condition=UnlessCondition(use_joint_state_sanitizer),
    )
    if motion_only:
        controller_config_msg = (
            "[arm_start.launch] Motion-only ADL controller profile enabled "
            "(share/adl_tasks/config/ros2_controllers_vbox_motion.yaml, "
            f"update_rate:={configured_update_rate} Hz, no gripper/twist/fault controller "
            "spawners, MoveIt gripper controller removed)."
        )
    else:
        controller_config_msg = (
            "[arm_start.launch] Using ADL controller config "
            f"(share/adl_tasks/config/ros2_controllers_vbox.yaml, update_rate:={configured_update_rate} Hz)."
        )
    controller_config_info = LogInfo(msg=controller_config_msg)
    fake_hw_warning = LogInfo(
        msg=(
            "[arm_start.launch] use_fake_hardware:=true. RViz/MoveIt will follow fake controller "
            "state instead of the physical Kinova arm."
        ),
        condition=IfCondition(use_fake_hardware),
    )

    controller_spawner_actions = [robot_traj_controller_spawner]
    if not motion_only:
        controller_spawner_actions.append(robot_pos_controller_spawner)
        controller_spawner_actions.append(robot_hand_controller_spawner)
        if fault_controller_spawner is not None:
            controller_spawner_actions.append(fault_controller_spawner)

    arm_controller_spawner_start = TimerAction(
        period=0.5,
        actions=[
            # [FLAG arm-vbox-controller-order] Start the joint-state broadcaster first, then bring
            # up the arm and auxiliary controllers. The latest VM logs showed trajectory control
            # sometimes activating before /joint_states existed, which adds startup jitter and
            # "empty JointState" noise before the first motion request.
            *controller_spawner_actions,
        ],
    )
    start_arm_controllers_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner_start],
        )
    )

    moveit_and_rviz_start = TimerAction(
        period=1.5,
        actions=[
            # [FLAG arm-vbox-startup-delay] Let the arm controllers settle after startup before
            # MoveIt and RViz construct their initial current-state view. In sanitizer mode this
            # also gives the relay time to publish a fresh non-empty joint-state sample before
            # move_group subscribes.
            move_group_node,
            rviz_node,
        ],
    )
    delay_moveit_and_rviz_after_joint_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_traj_controller_spawner,
            on_exit=[moveit_and_rviz_start],
        )
    )

    nodes_to_start = [
        state_source_info,
        moveit_topic_env_sanitized,
        moveit_topic_env_raw,
        controller_config_info,
        fake_hw_warning,
        ros2_control_node,
        robot_state_publisher,
        joint_state_sanitizer_node,
        joint_state_broadcaster_spawner,
        start_arm_controllers_after_joint_state_broadcaster,
        delay_moveit_and_rviz_after_joint_trajectory_spawner,
        *( [move_group_state_guard] if move_group_state_guard is not None else [] ),
        static_tf,
        *_wrist_camera_static_tf_nodes(condition=IfCondition(vision)),
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.10",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_sanitizer",
            default_value="true",
            description=(
                "Publish /joint_states_sanitized for ADL MoveIt helpers. Disable for raw-state "
                "viewer debugging."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motion_only_mode",
            default_value="false",
            description=(
                "Use the lean arm-only motion profile: lower controller rate and only arm "
                "trajectory/joint-state controllers."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_update_rate",
            default_value="",
            description=(
                "Override ros2_control update_rate in Hz. Leave empty to use the project defaults "
                "(50 in motion_only_mode, 125 for fake-hardware full-stack, 50 for real hardware). "
                "Explicit real-hardware overrides above 125 Hz are honored for tuning."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lock_memory",
            default_value="false",
            description=(
                "Pass lock_memory to ros2_control_node. Useful for cutting page-fault jitter if "
                "the guest is allowed to lock memory."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_thread_priority",
            default_value="50",
            description="Pass thread_priority to ros2_control_node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_cpu_affinity",
            default_value="",
            description=(
                "Optional CPU affinity for ros2_control_node, for example '2' or '2,3'. Leave "
                "empty to let the guest scheduler place it freely."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_print_overrun_warnings",
            default_value="true",
            description=(
                "Forward overruns.print_warnings to ros2_control_node. Disable only when you want "
                "to reduce warning spam during motion tuning runs."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            default_value="true",
            description="Include wrist camera links/frames in the robot description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use arm's internal gripper connection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_external_cable",
            default_value="false",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
