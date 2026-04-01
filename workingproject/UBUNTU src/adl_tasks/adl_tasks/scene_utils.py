import time
import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from adl_tasks.adl_config import (
    BOTTLE_RADIUS, BOTTLE_HEIGHT, BOTTLE_LENGTH_AXIS,
    MEDICATION_RADIUS, MEDICATION_HEIGHT,
    CUP_RADIUS, CUP_HEIGHT,
    REMOTE_WIDTH, REMOTE_LENGTH, REMOTE_THICKNESS,
    CUBE_SIZE,
    GRASP_CLEARANCE,
)
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.grasp_and_place import (
    compute_side_qr_face_standoff_m,
    grasp_axis_ee_to_pinch_center_m,
)

OBJECT_SHAPES = {
    0: {"shape": "cylinder", "radius": BOTTLE_RADIUS, "height": BOTTLE_HEIGHT},
    1: {"shape": "cylinder", "radius": MEDICATION_RADIUS, "height": MEDICATION_HEIGHT},
    2: {"shape": "cylinder", "radius": CUP_RADIUS, "height": CUP_HEIGHT},
    3: {"shape": "box", "sx": REMOTE_WIDTH, "sy": REMOTE_LENGTH, "sz": REMOTE_THICKNESS},
    4: {"shape": "box", "sx": CUBE_SIZE, "sy": CUBE_SIZE, "sz": CUBE_SIZE},
}

def _get_scene_pub(node):
    if not hasattr(node, "_scene_pub") or node._scene_pub is None:
        node._scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    return node._scene_pub

def _infer_tag_id(object_id: str):
    text = str(object_id)
    if text.startswith("obj_"):
        suffix = text.split("_", 1)[1]
        if suffix.isdigit():
            return int(suffix)
    return None


def _tag_axes(tag_pose: Pose):
    q = tag_pose.orientation
    rot = Rotation.from_quat([float(q.x), float(q.y), float(q.z), float(q.w)]).as_matrix()
    return rot[:, 0], rot[:, 1], rot[:, 2]


def _pose_axes(pose: Pose):
    q = pose.orientation
    rot = Rotation.from_quat([float(q.x), float(q.y), float(q.z), float(q.w)]).as_matrix()
    return rot[:, 0], rot[:, 1], rot[:, 2]


def _object_primitive_for_tag(tag_id: int):
    shape = OBJECT_SHAPES.get(tag_id)
    if shape is None:
        return None

    prim = SolidPrimitive()
    if shape["shape"] == "cylinder":
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [shape["height"], shape["radius"]]
    else:
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [shape["sx"], shape["sy"], shape["sz"]]
    return prim


def _attached_center_offset_m(tag_id: int):
    obj = OBJECTS.get(tag_id)
    if obj is None:
        return None

    grasp_axis = getattr(obj, "grasp_axis_size_m", None)
    if grasp_axis is None:
        return None

    carry_override = getattr(obj, "carry_center_offset_m", None)
    if carry_override is not None:
        try:
            # [FLAG attached-carry-override] Let an object override its carry center when
            # the attached shape needs to sit slightly differently than the nominal pick target.
            return float(carry_override)
        except (TypeError, ValueError):
            pass

    if getattr(obj, "approach_type", None) == "top":
        # Use the same top-grasp depth logic as the pick pose:
        # raw top grasp offset is EE-above-top-surface, so adding half thickness gives
        # the object center relative to the live EE origin along local +Z.
        return float(obj.grasp_offset[2] + 0.5 * float(grasp_axis))

    if getattr(obj, "approach_type", None) == "side":
        ee_to_pinch_center_m, _ = grasp_axis_ee_to_pinch_center_m(obj)
        if ee_to_pinch_center_m is not None:
            # Keep the carried-object center on the same measured grasp axis used to build
            # the side pick pose, instead of reconstructing carry geometry only from QR-face stand-off.
            return float(ee_to_pinch_center_m + 0.5 * float(GRASP_CLEARANCE))
        # Side grasp target is offset outward from the QR face, so center distance from the
        # live EE is "face stand-off + half object thickness along approach axis".
        face_standoff_m, _, _ = compute_side_qr_face_standoff_m(obj)
        return float(face_standoff_m + 0.5 * float(grasp_axis))

    return None


def _make_attached_collision_from_ee(
    object_id: str,
    tag_id: int,
    ee_pose: Pose,
    carry_orientation_mode: str | None = None,
):
    prim = _object_primitive_for_tag(tag_id)
    if prim is None:
        return None

    obj = OBJECTS.get(tag_id)
    center_offset_m = _attached_center_offset_m(tag_id)
    if obj is None or center_offset_m is None:
        return None

    ee_x, ee_y, ee_z = _pose_axes(ee_pose)
    center = Pose()
    center.position.x = float(ee_pose.position.x + ee_z[0] * center_offset_m)
    center.position.y = float(ee_pose.position.y + ee_z[1] * center_offset_m)
    center.position.z = float(ee_pose.position.z + ee_z[2] * center_offset_m)

    shape = OBJECT_SHAPES[tag_id]
    # [FLAG attached-from-live-ee] Attach from the live gripper pose instead of the original
    # tag pose so the carried-object scene matches the executed grasp for both stub and real vision.
    if shape["shape"] == "cylinder" and getattr(obj, "approach_type", None) == "side":
        # Side-grasped cylindrical objects should remain upright in the planning scene. The
        # carry pose is defined by the live EE position, but the cylinder axis stays world-up.
        center.orientation.x = 0.0
        center.orientation.y = 0.0
        center.orientation.z = 0.0
        center.orientation.w = 1.0
    elif (
        carry_orientation_mode == "top_cylinder_keep_horizontal"
        and shape["shape"] == "cylinder"
        and getattr(obj, "approach_type", None) == "top"
    ):
        # [FLAG attached-bottle-horizontal] Keep a top-grasped fallen bottle horizontal in the
        # carried planning-scene model. This is opt-in from the task layer so successful ADLs
        # keep their current attachment behavior unless they explicitly request this bottle mode.
        cyl_z = np.array(ee_y, dtype=float)
        cyl_z = cyl_z / (np.linalg.norm(cyl_z) + 1e-9)
        cyl_x = np.array(ee_x, dtype=float)
        cyl_x = cyl_x - cyl_z * np.dot(cyl_x, cyl_z)
        if np.linalg.norm(cyl_x) < 1e-6:
            cyl_x = np.array(ee_z, dtype=float)
            cyl_x = cyl_x - cyl_z * np.dot(cyl_x, cyl_z)
        cyl_x = cyl_x / (np.linalg.norm(cyl_x) + 1e-9)
        cyl_y = np.cross(cyl_z, cyl_x)
        cyl_y = cyl_y / (np.linalg.norm(cyl_y) + 1e-9)
        rot = np.column_stack((cyl_x, cyl_y, cyl_z))
        u, _, vt = np.linalg.svd(rot)
        rot = u @ vt
        if np.linalg.det(rot) < 0:
            rot[:, -1] *= -1
        q = Rotation.from_matrix(rot).as_quat()
        center.orientation.x = float(q[0])
        center.orientation.y = float(q[1])
        center.orientation.z = float(q[2])
        center.orientation.w = float(q[3])
    else:
        center.orientation = ee_pose.orientation

    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = str(object_id)
    co.primitives = [prim]
    co.primitive_poses = [center]
    co.operation = CollisionObject.ADD
    return co


def _make_attached_collision_object(object_id: str, tag_id: int, tag_pose: Pose):
    prim = _object_primitive_for_tag(tag_id)
    shape = OBJECT_SHAPES.get(tag_id)
    if prim is None or shape is None:
        return None

    tag_x, tag_y, tag_z = _tag_axes(tag_pose)
    cx = float(tag_pose.position.x)
    cy = float(tag_pose.position.y)
    cz = float(tag_pose.position.z)

    oriented_pose = Pose()
    if tag_id == 0:
        r = shape["radius"]
        oriented_pose.position.x = cx - tag_z[0] * r
        oriented_pose.position.y = cy - tag_z[1] * r
        oriented_pose.position.z = cz - tag_z[2] * r

        length_axis = tag_y if BOTTLE_LENGTH_AXIS == "y" else tag_x
        cyl_z = length_axis
        cyl_x = tag_z
        cyl_y = np.cross(cyl_z, cyl_x)
        if np.linalg.norm(cyl_y) < 1e-6:
            cyl_x = tag_x
            cyl_y = np.cross(cyl_z, cyl_x)
        cyl_y = cyl_y / np.linalg.norm(cyl_y)
        cyl_x = np.cross(cyl_y, cyl_z)
        cyl_x = cyl_x / np.linalg.norm(cyl_x)

        rot = np.column_stack((cyl_x, cyl_y, cyl_z))
        u, _, vt = np.linalg.svd(rot)
        rot = u @ vt
        if np.linalg.det(rot) < 0:
            rot[:, -1] *= -1
        q = Rotation.from_matrix(rot).as_quat()
        oriented_pose.orientation.x = float(q[0])
        oriented_pose.orientation.y = float(q[1])
        oriented_pose.orientation.z = float(q[2])
        oriented_pose.orientation.w = float(q[3])
    elif tag_id in (1, 2):
        r = shape["radius"]
        oriented_pose.position.x = cx - tag_z[0] * r
        oriented_pose.position.y = cy - tag_z[1] * r
        oriented_pose.position.z = cz - tag_z[2] * r
        oriented_pose.orientation.w = 1.0
    elif tag_id == 3:
        tag_from_end = 0.01
        half_len = shape["sy"] / 2.0
        half_thk = shape["sz"] / 2.0
        length_axis = tag_y
        oriented_pose.position.x = cx + length_axis[0] * (half_len - tag_from_end) - tag_z[0] * half_thk
        oriented_pose.position.y = cy + length_axis[1] * (half_len - tag_from_end) - tag_z[1] * half_thk
        oriented_pose.position.z = cz + length_axis[2] * (half_len - tag_from_end) - tag_z[2] * half_thk
        oriented_pose.orientation = tag_pose.orientation
    elif tag_id == 4:
        half = shape["sz"] / 2.0
        oriented_pose.position.x = cx - tag_z[0] * half
        oriented_pose.position.y = cy - tag_z[1] * half
        oriented_pose.position.z = cz - tag_z[2] * half
        oriented_pose.orientation = tag_pose.orientation
    else:
        oriented_pose.position.x = cx
        oriented_pose.position.y = cy
        oriented_pose.position.z = cz
        oriented_pose.orientation.w = 1.0

    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = str(object_id)
    co.primitives = [prim]
    co.primitive_poses = [oriented_pose]
    co.operation = CollisionObject.ADD
    return co

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

def attach_object(
    node,
    object_id: str,
    link_name: str,
    touch_links: list,
    tag_id: int = None,
    tag_pose: Pose = None,
    carry_orientation_mode: str | None = None,
):
    pub = _get_scene_pub(node)

    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.touch_links = touch_links
    
    resolved_tag_id = tag_id if tag_id is not None else _infer_tag_id(object_id)
    collision_object = None
    if resolved_tag_id is not None:
        ee_pose = None
        arm = getattr(node, "arm", None)
        if arm is not None and hasattr(arm, "get_current_end_effector_pose"):
            try:
                ee_pose = arm.get_current_end_effector_pose(timeout=1.0)
            except Exception as exc:
                node.get_logger().warn(
                    f"attach_object: failed to read live EE pose for {object_id}: {exc}"
                )
        if ee_pose is not None:
            collision_object = _make_attached_collision_from_ee(
                object_id=object_id,
                tag_id=int(resolved_tag_id),
                ee_pose=ee_pose,
                carry_orientation_mode=carry_orientation_mode,
            )
        if collision_object is None and tag_pose is not None:
            # Fallback: still allow tag-based reconstruction if FK/live EE is unavailable.
            collision_object = _make_attached_collision_object(
                object_id=object_id,
                tag_id=int(resolved_tag_id),
                tag_pose=tag_pose,
            )

    if collision_object is not None:
        # When the world object was removed before grasp, include the primitive
        # geometry here so MoveIt can reason about the carried object during lift.
        aco.object = collision_object
    else:
        aco.object.header.frame_id = "base_link"
        aco.object.id = str(object_id)
        aco.object.operation = aco.object.ADD
        node.get_logger().warn(
            f"attach_object: attaching {object_id} without explicit geometry "
            f"(tag_id={resolved_tag_id}, tag_pose={'set' if tag_pose is not None else 'missing'})."
        )
    
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.attached_collision_objects = [aco]
    scene.robot_state.is_diff = True

    pub.publish(scene)
    # log if geometry was included
    if collision_object is not None:
        node.get_logger().info(
            f"Attached object {object_id} to {link_name} with geometry for tag ID {resolved_tag_id}."
        )
    else:
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
