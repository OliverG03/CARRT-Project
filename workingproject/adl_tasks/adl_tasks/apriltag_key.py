# NONE placeholder until locations are imported from AprilTag vision node

# Define a Master Key (class) to store AprilTag IDs and corresponding information on objects:
# - AprilTag ID
# - Object name
# - Object type (e.g., "bottle", "medication", etc.)
# - Task relevance
# - Destination information (e.g., "shelf", "bin", etc.)
# - Any other relevant metadata for task execution and planning.

import clear_table, give_medication, pick_dropped_bottle
import vision_apriltag


class AprilTagObject:
    def __init__(self, name, ID, object_type, adl_used, grasp_offset, approach_vector, gripper_width, destination):
        self.name = name
        self.ID = ID
        self.object_type = object_type
        self.adl_used = adl_used
        self.grasp_offset = grasp_offset
        self.approach_vector = approach_vector
        self.gripper_width = gripper_width
        self.destination = destination

    def compute_grasp_pose(self, tag_pose):
        """Compute the grasp pose based on the tag pose and the stored grasp offset and approach vector."""
        # This is a placeholder for the actual computation logic.
        # In a real implementation, you'd use the tag_pose, grasp_offset, and approach_vector
        # to calculate the final grasp pose for the robot.
        grasp_pose = tag_pose  # Replace with actual computation
        return grasp_pose
    
# define a key to map AprilTag IDs to their corresponding drop-off locations or other relevant metadata for task execution
# - QR codes placed where objects should be dropped off (e.g., "shelf 1", "bin", etc.)
class AprilTagLocation:
    def __init__(self, name, ID, location_type, coordinates):
        self.name = name
        self.ID = ID
        self.coordinates = coordinates # get when AprilTag evaluated
    
LOCATIONS = {
    # drop off for medication bottle (shelf, spot 1)
    1: AprilTagLocation(
        name="Shelf 1",
        ID=1,
        self.coordinates=None
    ),
    # drop off for water bottle (near user)
    2: AprilTagLocation(
        name="Near User",
        ID=2,
        self.coordinates=None
    ),
    # drop off for household objects
    3: AprilTagLocation(
        name="Shelf 2",
        ID=3,
        self.coordinates=None
    ),
    4: AprilTagLocation(
        name="Bin",
        ID=4,
        self.coordinates=None
    )
}
    
# define objects with their corresponding AprilTag IDs and metadata
OBJECTS = {
    # water bottle (ADL task 1: pick up and replace water bottle)
    1: AprilTagObject(
        name="Water Bottle",
        ID=1,
        object_type="bottle",
        adl_used=pick_dropped_bottle,
        grasp_offset=[0, 0, 0.1],  # Example offset
        approach_vector=[0, 0, -1],  # Approach from above
        gripper_width=0.05,  # Example gripper width
        destination="Near User"
    ),
    # medication bottle (ADL task 2: give medication to user)
    2: AprilTagObject(
        name="Medication Bottle",
        ID=2,
        object_type="medication",
        adl_used=give_medication,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        gripper_width=0.03,
        destination="shelf 1"
    ),
    # household objects (ADL task 2: clear household objects)
    3: AprilTagObject(
        name="Cup",
        ID=3,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        gripper_width=0.04,
        destination="Shelf 2"
    ),
    4: AprilTagObject(
        name="Toy Car",
        ID=4,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        gripper_width=0.06,
        destination="Bin"
    ),
    5: AprilTagObject(
        name="Rubik's Cube",
        ID=5,
        object_type="Household Object",
        adl_used=clear_table,
        grasp_offset=[0, 0, 0.05],
        approach_vector=[0, 0, -1],
        gripper_width=0.05,
        destination="Bin"
    )
}