#!/usr/bin/env python3
"""
Utility script to collect and log bottle scan poses for the pick_dropped_bottle task.

This script helps you manually command poses and capture the end-effector transforms,
so you can update the baseline or sweep scan poses with better orientations/positions.

Usage:
    ros2 run adl_tasks collect_bottle_scan_poses
    
    Or from command line with joint positions:
    ros2 run adl_tasks collect_bottle_scan_poses -- --joints "j1=0.1,j2=-1.5,j3=0.0,j4=-1.2,j5=0.0,j6=0.5,j7=0.0"

Commands (interactive mode):
    home         - Move to home
    table        - Move to look_at_table
    ground       - Move to look_at_ground  
    custom j1 j2 j3 j4 j5 j6 j7  - Move to custom joint positions
    current      - Print current end-effector pose
    save <name>  - Save current pose as <name>
    poses        - List all saved poses
    export       - Export saved poses in LOOK_AT_GROUND_JOINTS format
    quit         - Exit
"""

import rclpy
from rclpy.node import Node
import json
import sys

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.adl_logging import log_pose

class BottleScanPoseCollector(Node):
    def __init__(self):
        super().__init__('bottle_scan_pose_collector')
        self.arm = MoveItHelper(self)
        self.saved_poses = {}
        
    def get_current_pose(self):
        """Get the current end-effector pose."""
        try:
            return self.arm.get_current_pose()
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None
    
    def get_current_joints(self):
        """Get the current joint positions."""
        try:
            return self.arm.get_current_joints()
        except Exception as e:
            self.get_logger().error(f"Failed to get current joints: {e}")
            return None
    
    def move_to_joints(self, joint_dict):
        """Move to specified joint positions."""
        try:
            result = self.arm.go_to_joint_positions(joint_dict)
            if result:
                self.get_logger().info(f"Successfully moved to joints: {joint_dict}")
                self.print_current_state()
            else:
                self.get_logger().warn("Move failed")
            return result
        except Exception as e:
            self.get_logger().error(f"Move failed: {e}")
            return False
    
    def print_current_state(self):
        """Print current end-effector pose and joints."""
        pose = self.get_current_pose()
        joints = self.get_current_joints()
        
        if pose:
            self.get_logger().info(f"End-Effector Pose:")
            self.get_logger().info(f"  Position: [{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}]")
            self.get_logger().info(f"  Orientation (quat xyzw): [{pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f}]")
            
            # Convert to RPY
            rot = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            rpy = rot.as_euler('xyz')
            self.get_logger().info(f"  Orientation (RPY rad): [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")
            self.get_logger().info(f"  Orientation (RPY deg): [{float(rpy[0])*180/3.14159:.1f}, {float(rpy[1])*180/3.14159:.1f}, {float(rpy[2])*180/3.14159:.1f}]")
        
        if joints:
            self.get_logger().info(f"Joint Positions: {joints}")
    
    def save_current_pose(self, name):
        """Save current pose with a name."""
        joints = self.get_current_joints()
        pose = self.get_current_pose()
        
        if joints and pose:
            self.saved_poses[name] = {
                "joints": dict(joints),
                "pose": {
                    "position": [pose.position.x, pose.position.y, pose.position.z],
                    "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                }
            }
            self.get_logger().info(f"Saved pose as '{name}'")
            return True
        return False
    
    def list_saved_poses(self):
        """List all saved poses."""
        if not self.saved_poses:
            self.get_logger().info("No poses saved yet")
            return
        
        self.get_logger().info("Saved poses:")
        for name, data in self.saved_poses.items():
            joints = data["joints"]
            pos = data["pose"]["position"]
            self.get_logger().info(f"  {name}:")
            self.get_logger().info(f"    Joints: {joints}")
            self.get_logger().info(f"    Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    
    def export_poses(self):
        """Export poses in helper_moves format."""
        output = "LOOK_AT_GROUND_JOINTS = {\n"
        for name, data in self.saved_poses.items():
            joints = data["joints"]
            output += f'    # {name}\n'
            output += '    "' + name + '": {\n'
            for joint_name, value in joints.items():
                output += f'        "{joint_name}": {value},\n'
            output += '    },\n'
        output += "}\n"
        
        self.get_logger().info("Export format:\n" + output)
        
        # Also save to file
        filename = "/tmp/bottle_scan_poses.py"
        with open(filename, 'w') as f:
            f.write(output)
        self.get_logger().info(f"Saved to {filename}")

def main():
    rclpy.init()
    collector = BottleScanPoseCollector()
    
    try:
        print("\n=== Bottle Scan Pose Collector ===")
        print("Commands: home, table, ground, current, save <name>, poses, export, quit")
        print("Or: custom j1 j2 j3 j4 j5 j6 j7")
        print()
        
        while True:
            try:
                cmd = input("Enter command: ").strip().split()
                
                if not cmd:
                    continue
                
                if cmd[0] == "quit" or cmd[0] == "exit":
                    break
                elif cmd[0] == "home":
                    collector.arm.go_home()
                    collector.print_current_state()
                elif cmd[0] == "table":
                    collector.arm.look_at_table()
                    collector.print_current_state()
                elif cmd[0] == "ground":
                    collector.arm.look_at_ground()
                    collector.print_current_state()
                elif cmd[0] == "current":
                    collector.print_current_state()
                elif cmd[0] == "save" and len(cmd) > 1:
                    collector.save_current_pose(cmd[1])
                elif cmd[0] == "poses":
                    collector.list_saved_poses()
                elif cmd[0] == "export":
                    collector.export_poses()
                elif cmd[0] == "custom" and len(cmd) == 8:
                    try:
                        joints = {
                            "joint_1": float(cmd[1]),
                            "joint_2": float(cmd[2]),
                            "joint_3": float(cmd[3]),
                            "joint_4": float(cmd[4]),
                            "joint_5": float(cmd[5]),
                            "joint_6": float(cmd[6]),
                            "joint_7": float(cmd[7]),
                        }
                        collector.move_to_joints(joints)
                    except ValueError:
                        print("Joint values must be floats")
                else:
                    print("Unknown command. Try: home, table, ground, current, save <name>, poses, export, quit")
                    print("Or: custom j1 j2 j3 j4 j5 j6 j7")
                
                rclpy.spin_once(collector, timeout_sec=0.1)
            except KeyboardInterrupt:
                break
            except Exception as e:
                collector.get_logger().error(f"Error: {e}")
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
