# adl_sim.launch.py
# master launch file for ADL simulation testing
# Starts: MoveIt, Vision Stub, and all ADL tasks together

# Usage:
# - terminal 1: 
#       ros2 launch kortex_bringup kortex_sim_control.launch.py sim_gazebo:=true vision:=true gripper:=robotiq_2f_85 launch_rviz:=false
# - terminal 2: 
#       ros2 launch adl_tasks adl_sim.launch.py

# import necessary ROS2 launch libraries
from launch import LaunchDescription                    # define launch description
from launch.actions import DeclareLaunchArgument        # declare launch arguments
from launch.substitutions import LaunchConfiguration    # access launch arguments
from launch_ros.actions import Node                     # define ROS2 nodes
from launch.conditions import IfCondition               # conditionally launch nodes

# --- Define the launch description --- #
def generate_launch_description():
    use_stub = DeclareLaunchArgument(
        'use_stub',
        default_value='true',
        description='Launch vision stub node instead of real vision node'
    )
    launch_ui = DeclareLaunchArgument(
        'launch_ui',
        default_value='true',
        description='Launch web UI node'
    )
    
    # vision stub node (swap to vision_apriltag for real hardware)
    vision_stub_node = Node(
        package='adl_tasks',
        executable='vision_stub',
        name='vision_stub_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_stub'))
    )
    
    '''
    # real vision node (only runs if use_stub is false)
    vision_node = Node(
        package='adl_tasks',
        executable='vision_apriltag',
        name='vision_apriltag_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_stub'))
    )
    '''
    
    # --- ADL TASKS --- #
    pick_bottle_node = Node(
        package='adl_tasks',
        executable='pick_dropped_bottle',
        name='pick_dropped_bottle_node',
        output='screen'
    )
    
    clear_table_node = Node(
        package='adl_tasks',
        executable='clear_table',
        name='clear_table_node',
        output='screen'
    )
    
    give_medication_node = Node(
        package='adl_tasks',
        executable='give_medication',
        name='give_medication_node',
        output='screen'
    )
    
    # --- Web UI --- #
    ui_node = Node(
        package='adl_tasks',
        executable='adl_ui',
        name='adl_ui_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_ui'))
    )
    
    return LaunchDescription([
        use_stub,
        launch_ui,
        vision_stub_node,
        # vision_node,
        pick_bottle_node,
        clear_table_node,
        give_medication_node,
        ui_node,
    ])