import time
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from std_msgs.msg import Header

def _get_scene_pub(node):
    if not hasattr(node, "_scene_pub") or node._scene_pub is None:
        node._scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    return node._scene_pub

def remove_collision_object(node, object_id: str):
    pub = _get_scene_pub(node)

    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = object_id
    co.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [co]

    pub.publish(scene)
    node.get_logger().info(f"Removed collision object {object_id} from planning scene.")
    time.sleep(0.35)

def attach_object(node, object_id: str, link_name: str, touch_links: list):
    pub = _get_scene_pub(node)

    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.object.header.frame_id = "base_link"
    aco.object.id = str(object_id)
    aco.object.operation = aco.object.ADD
    aco.touch_links = touch_links

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.attached_collision_objects = [aco]
    scene.robot_state.is_diff = True

    pub.publish(scene)
    node.get_logger().info(f"Attached object {object_id} to {link_name}.")
    time.sleep(0.2)

def detach_object(node, object_id: str, link_name: str):
    pub = _get_scene_pub(node)

    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.object.header.frame_id = "base_link"
    aco.object.id = str(object_id)
    aco.object.operation = aco.object.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.attached_collision_objects = [aco]
    scene.robot_state.is_diff = True

    pub.publish(scene)
    node.get_logger().info(f"Detached object {object_id} from {link_name}.")
    time.sleep(0.2)