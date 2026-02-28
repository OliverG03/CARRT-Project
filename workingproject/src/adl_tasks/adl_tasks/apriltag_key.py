# apriltag_key.py

# NONE placeholder until locations are imported from AprilTag vision node

# Define a Master Key (class) to store AprilTag IDs and corresponding information on objects:
# - AprilTag ID
# - Object name
# - Object type (e.g., "bottle", "medication", etc.)
# - Task relevance
# - Destination information (e.g., "shelf", "bin", etc.)
# - Any other relevant metadata for task execution and planning.

from geometry_msgs.msg import Pose


# MAX WIDTH = 0.085 m (fully open gripper)
BOTTLE_DIAMETER = 0.067 # m
MEDICATION_DIAMETER = 0.06 # m
CUP_DIAMETER = 0.075 # m
REMOTE_WIDTH = 0.043 # m
CUBE_SIZE = 0.065 # m

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
        # TODO: derive orientation from approach vector
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
# OBJECTS: 0-4
# LOCATIONS: 5-8

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
    # -- water bottle (ADL task 1: pick up and replace water bottle)
    # tag on side of bottle (4x) -> facing robot (upwards)
    # approach from ABOVE
    0: AprilTagObject(
        name="Water Bottle",
        ID=0,
        object_type="bottle",
        adl_used="pick_dropped_bottle",
        
        grasp_offset=[0.05, 0, 0],
        approach_vector=[0, 0, -1],  # approach from above
        
        gripper_width=BOTTLE_DIAMETER + 0.01, # add some tolerance (consider if necessary)
        gripper_force=15,
        gripper_speed=0.03, # slow, avoid rolling
        
        destination=5 # hand off near user
    ),
    # medication bottle (ADL task 2: give medication to user)
    # tag on side of bottle (>=2x) -> facing robot horizontally
    # approach from the SIDE
    1: AprilTagObject(
        name="Medication Bottle",
        ID=1,
        object_type="medication",
        adl_used="give_medication",
        grasp_offset=[0, 0, 0.04], 
        approach_vector=[1, 0, 0], # horizontal approach
        
        gripper_width=MEDICATION_DIAMETER + 0.01, # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=5 # hand off near user
    ),
    
    # household objects (ADL task 3: clear household objects)
    # upright facing robot
    # approach horizontally
    2: AprilTagObject(
        name="Cup",
        ID=2,
        object_type="Household Object",
        adl_used="clear_table",
        grasp_offset=[0, 0, 0.04],
        approach_vector=[1, 0, 0],
        
        gripper_width=CUP_DIAMETER + 0.01, # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=6 # Shelf 1
    ),
    # tag on top of remote, facing up
    # approach from ABOVE
    3: AprilTagObject(
        name="TV Remote",
        ID=3,
        object_type="Household Object",
        adl_used="clear_table",
        grasp_offset=[0, 0, 0.02],
        approach_vector=[0, 0, -1],
        
        gripper_width=REMOTE_WIDTH + 0.01, # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=7 # Shelf 2
    ),
    # tag on top of cube, facing up
    # approach from ABOVE
    4: AprilTagObject(
        name="Cube",
        ID=4,
        object_type="Household Object",
        adl_used="clear_table",
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        
        gripper_width=CUBE_SIZE + 0.01, # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=8 # Bin
    ),
}