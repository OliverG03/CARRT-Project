import rclpy #jus
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import numpy as np
import copy
from apriltag_key import OBJECTS, LOCATIONS

bottle = OBJECTS[0]  #pre configured bottle object
destination = LOCATIONS[bottle.destination] #destination for bottle placement

class PickDroppedBottle(Node):
    #--------------------STATES------------------#
    IDLE = "IDLE" #task not started
    SEARCH = "SEARCH"#arm sweeping to find bottle
    DETECTED = "DETECTED"#bottle tag seen and pose stored
    PICKING = "PICKING"# executing grasp sequence
    DELIVERING = "DELIVERING"#moving to drop-off location
    DONE = "DONE"#task complete, node stops


    def __init__(self):
        super().__init__('picked_dropped_bottle') #registers node


        #--TF2 setup--#
        self.tf_buffer =tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--AprilTag subscriber --#
        self.tag_sub =self.create_subscription(AprilTagDetectionArray,'/apriltag/detections',self.tag_callback,10)


        #-- State tracking --#
        self.state = self.IDLE
        self.bottle_pose = None #PoseStamped in base_link frame, filled by tag_callback
        self.dest_pose = None #destination coordinates, filled when location tag is seen

        self.timer = self.create_time(0.1,self.run)

        self.get_logger().info("PickedDroppedBottle node ready.")


        def tag_callback(self,msg):
            for detection in msg.detections:
                tag_id = detection.id[0]

                if tag_id == bottle.ID and self.bottle_pose is None:
                    raw_pose = detection.pose.pose.pose
                    self.bottle_pose = self.transform_to_base(raw_pose)
                    self.get_logger().info(f"Bottle detected at: {self.bottle_pose}")

                if tag_id == destination.id and self.dest_pose is None:
                    raw_pose = detection.pose.pose.pose
                    self.dest_pose = destination.get_coordinates(raw_pose)    

        def transform_to_base(self,pose):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "camera_color_optical_frame"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose

        
        try:
            return self.tf_buffer.transform(pose_stamped,"base_link",timeout=rclpy.duration.Duration(seconds=1.0))
        
        except Exception as e:
            self.get_logger().warn(f"TF transformed failed: {e}")
            return None
        
        
        def run(self):

            if self.state == self.IDLE:
                self.get_logger().info("Starting task: Picked Dropped Bottle")
                self.state = self.SEARCH

            elif self.state == self.SEARCH:
                self.search_for_bottle()
            
            elif self.state == self.SEARCH:
                self.execute_pick()

            elif self.state == self.SEARCH:
                self.execute_deliver()

            elif self.state == self.DONE:
                self. get_logger().info("TASK COMPLETE")
                self.timer.cancel()

        def search_for_bottle(self):

            if self.bottle_pose is not None:
                self.get_logger().info("Bottle found - transitioning states")
                self.state = self.DETECTED
                return
            

            #TODO: command arm through predefined list of joint configs that sweep the camera acorss the expected search area

            self.get_logger().info("Searching for bottle....")


        def execute_pick(self):

            if self.bottle_pose is None:
                self.state = self.SEARCH
                return

        #####-Hover above grasp point-#############
            grasp_pose = bottle.compute_grasp_pose(self.bottle_pose.pose) 

            hover_pose = copy.deepcopy(grasp_pose)
            hover_pose.position.z += 0.15
            self.get_logger().info("Moving to hover position...")


        #######-Open gripper-############
        # gripper width from apriltag_key = bottle diameter + 1 cm tolerance
            self.get_logger().info(f"Opening gripper to {bottle.gripper_width}m")
            #self.gripper.open(bottle.gripper_width)


       # --Step 3: Descend to grasp --#
            self.get_logger().info("Descending to grasp...")
        

                # -- Step 4: Close gripper --
        # Uses bottle-specific force (15N) and slow speed (0.03 m/s)
        # Slow speed prevents the cylindrical bottle from rolling away
        self.get_logger().info(
            f"Closing gripper — "
            f"force: {BOTTLE.gripper_force}N  "
            f"speed: {BOTTLE.gripper_speed}m/s"
        )
        # self.gripper.close(
        #     force=BOTTLE.gripper_force,
        #     speed=BOTTLE.gripper_speed
        # )

        # -- Step 5: Lift --
        # deep copy grasp_pose again for the lift target
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += 0.20               # 20cm lift clearance
        self.get_logger().info("Lifting bottle...")
        # self.motion_planner.move_to_pose(lift_pose)

        # Grasp complete — move to delivery
        self.state = self.DELIVERING


    # -----------------------------------------------------------------------
    # execute_deliver: move to drop-off location and release the bottle
    #
    # The destination pose comes from the LOCATIONS tag (ID 5 = "Near User")
    # seen by tag_callback. If it hasn't been seen yet, we wait here.
    # -----------------------------------------------------------------------
    def execute_deliver(self):

        # Wait until the destination tag has been detected
        if self.dest_pose is None:
            self.get_logger().warn("Destination tag not yet seen — waiting...")
            return

        # Move to the drop-off location
        self.get_logger().info(f"Delivering to: {DESTINATION.name}")
        # self.motion_planner.move_to_pose(self.dest_pose)

        # Release the bottle
        # self.gripper.open(BOTTLE.gripper_width)

        # Return arm to home/rest position
        # self.motion_planner.move_to_home()

        self.state = self.DONE


# -----------------------------------------------------------------------
# Entry point — run this file directly or via a ROS 2 launch file
# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)                    # initialize ROS 2 runtime
    node = PickDroppedBottle()               # create and start the node
    rclpy.spin(node)                         # keep node alive until Ctrl+C
    rclpy.shutdown()                         # clean up ROS 2 on exit

if __name__ == '__main__':
    main()
        
    