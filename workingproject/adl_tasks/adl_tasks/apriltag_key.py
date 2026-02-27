# apriltag_key.py
# QR Code recognition
# NONE placeholder until locations are imported from AprilTag vision node

# Define a Master Key (class) to store AprilTag IDs and corresponding information on objects:
# - AprilTag ID
# - Object name
# - Object type (e.g., "bottle", "medication", etc.)
# - Task relevance
# - Destination information (e.g., "shelf", "bin", etc.)
# - Any other relevant metadata for task execution and planning.

from geometry_msgs.msg import Pose

class AprilTagObject:
    def __init__(self, name, ID, object_type, adl_used, 
                 grasp_offset, approach_vector, gripper_width, 
                 gripper_force, gripper_speed,destination):
        self.name = name
        self.ID = ID
        self.object_type = object_type  ### consider if necessary
        self.adl_used = adl_used        ### consider if necessary
        self.grasp_offset = grasp_offset            # [x,y,z] -> offset from tag pose to grasp point
        self.approach_vector = approach_vector      # direction to approach from
        self.gripper_width = gripper_width          # max: 0.085 = fully open
        self.gripper_force = gripper_force          # Newtons
        self.gripper_speed = gripper_speed          # m/s (max: 0.101)
        self.destination = destination              # AprilTag ID value for drop-off location

    def compute_grasp_pose(self, tag_pose):
        """Compute the grasp pose based on the tag pose and the stored grasp offset and approach vector."""
        # tag pose: position (x,y,z) and orientation (quaternion)\
        # returned from vision node
        grasp = Pose()
        grasp.position.x = tag_pose.position.x + self.grasp_offset[0]
        grasp.position.y = tag_pose.position.y + self.grasp_offset[1]
        grasp.position.z = tag_pose.position.z + self.grasp_offset[2]
        grasp.orientation = tag_pose.orientation
        return grasp
    
# define a key to map AprilTag IDs to their corresponding drop-off locations or other relevant metadata for task execution
# - QR codes placed where objects should be dropped off (e.g., "shelf 1", "bin", etc.)
class AprilTagLocation:
    def __init__(self, name, ID, location_type):
        self.name = name
        self.ID = ID
        self.location_type = location_type
        self.coordinates = None # get at runtime by vision node
        
    def get_coordinates(self, tag_pose):
        """Compute the location coordinates based on the tag pose."""
        # This is a placeholder for the actual computation logic.
        # In a real implementation, you'd use the tag_pose to determine the exact coordinates of the location.
        self.coordinates = tag_pose.position
        return self.coordinates
    
    
### Both locations and objects will use the same QR code detection system
# - therefore, their IDs must be unique across both categories


LOCATIONS = {
    # drop off for water bottle / medication (near user, edge of table, etc.)
    5: AprilTagLocation("Near User",    5, "handover"),
    # drop off for medication bottle (shelf, spot 1)
    6: AprilTagLocation("Shelf 1",      6, "shelf"),
    # drop off for household objects
    7: AprilTagLocation("Shelf 2",      7, "shelf"),
    8: AprilTagLocation("Bin",          8, "bin")
}
    
# define objects with their corresponding AprilTag IDs and metadata
OBJECTS = {
    # water bottle (ADL task 1: pick up and replace water bottle)
    1: AprilTagObject(
        name="Water Bottle",
        ID=1,
        object_type="bottle",
        adl_used=pick_dropped_bottle,
        
        grasp_offset=[0, 0, 0.1],
        approach_vector=[0, 0, -1],  # Approach from above
        
        gripper_width=0.067, 
        gripper_force=20,   # metal, can take force
        gripper_speed=0.05,
        
        destination=5 # hand off near user
    ),
    # medication bottle (ADL task 2: give medication to user)
    2: AprilTagObject(
        name="Medication Bottle",
        ID=2,
        object_type="medication",
        adl_used=give_medication,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        
        gripper_width=0.06,
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=5 # hand off near user
    ),
    
    # household objects (ADL task 2: clear household objects)
    3: AprilTagObject(
        name="Cup",
        ID=3,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        
        gripper_width=0.075,
        gripper_force=10,
        gripper_speed=0.05,
        
        destination=6
    ),
    4: AprilTagObject(
        name="TV Remote",
        ID=4,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        
        gripper_width=0.043,
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=7
    ),
    5: AprilTagObject(
        name="Cube",
        ID=5,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        
        gripper_width=0.065,
        gripper_force=15,
        gripper_speed=0.05,
        
        destination=8
    )
    
}
