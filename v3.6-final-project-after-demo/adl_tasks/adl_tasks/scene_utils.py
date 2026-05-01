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
    TABLE_X, TABLE_Y, TABLE_POS_X, TABLE_POS_Y, TABLE_SURFACE_Z,
)
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.grasp_and_place import (
    compute_side_qr_face_standoff_m,
    grasp_axis_ee_to_pinch_center_m,
)
from adl_tasks.adl_logging import pose_str

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
            # Let an object override its carry center when
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
    # Attach from the live gripper pose instead of the original
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
        # Keep a top-grasped fallen bottle horizontal in the
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


def upsert_collision_object_from_tag_pose(
    node,
    *,
    object_id: str,
    tag_id: int,
    tag_pose: Pose,
) -> bool:
    """
    Refresh or add a world collision object directly from a newly accepted tag pose.
    This is useful when a close-range QR alignment pass should replace a stale scan-memory pose.
    """
    pub = _get_scene_pub(node)
    collision_object = _make_attached_collision_object(
        object_id=object_id,
        tag_id=int(tag_id),
        tag_pose=tag_pose,
    )
    if collision_object is None:
        node.get_logger().warn(
            f"upsert_collision_object_from_tag_pose: failed to build collision geometry for "
            f"{object_id} (tag_id={tag_id})."
        )
        return False

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [collision_object]
    pub.publish(scene)
    node.get_logger().info(
        f"Updated planning-scene object {object_id} from accepted live tag pose (tag_id={tag_id})."
    )
    time.sleep(0.15)
    return True


def upsert_world_collision_object_from_pose(
    node,
    *,
    object_id: str,
    tag_id: int,
    pose: Pose,
) -> bool:
    """
    Add or refresh a world collision object directly from a supplied world pose.
    Useful after release when the object should immediately exist in the planning scene
    again before the arm retreats away from it.
    """
    pub = _get_scene_pub(node)
    prim = _object_primitive_for_tag(int(tag_id))
    shape = OBJECT_SHAPES.get(int(tag_id))
    if prim is None or shape is None:
        node.get_logger().warn(
            f"upsert_world_collision_object_from_pose: unknown collision geometry for "
            f"{object_id} (tag_id={tag_id})."
        )
        return False

    world_pose = Pose()
    world_pose.position.x = float(pose.position.x)
    world_pose.position.y = float(pose.position.y)
    world_pose.position.z = float(pose.position.z)
    if shape["shape"] == "cylinder" and int(tag_id) in (1, 2):
        # Keep medication/cup world objects upright after placement/release.
        world_pose.orientation.x = 0.0
        world_pose.orientation.y = 0.0
        world_pose.orientation.z = 0.0
        world_pose.orientation.w = 1.0
    else:
        world_pose.orientation = pose.orientation

    collision_object = CollisionObject()
    collision_object.header = Header()
    collision_object.header.frame_id = "base_link"
    collision_object.id = str(object_id)
    collision_object.primitives = [prim]
    collision_object.primitive_poses = [world_pose]
    collision_object.operation = CollisionObject.ADD

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [collision_object]
    pub.publish(scene)
    node.get_logger().info(
        f"Published world collision object {object_id} from explicit pose "
        f"(tag_id={tag_id}, pose={pose_str(world_pose)})."
    )
    time.sleep(0.15)
    return True

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
            node.get_logger().info(
                f"attach_object: live EE pose for {object_id}: {pose_str(ee_pose)}"
            )
        elif tag_pose is not None:
            node.get_logger().info(
                f"attach_object: falling back to tag pose for {object_id}: {pose_str(tag_pose)}"
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
            f"Attached object {object_id} to {link_name} with geometry for tag ID {resolved_tag_id} "
            f"(touch_links={list(touch_links)})."
        )
    else:
        node.get_logger().info(
            f"Attached object {object_id} to {link_name} without geometry "
            f"(touch_links={list(touch_links)})."
        )
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


def _make_box_collision_object(
    object_id: str,
    x_size: float,
    y_size: float,
    z_size: float,
    pos_x: float,
    pos_y: float,
    pos_z: float,
):
    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = str(object_id)

    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(x_size), float(y_size), float(z_size)]

    pose = Pose()
    pose.position.x = float(pos_x)
    pose.position.y = float(pos_y)
    pose.position.z = float(pos_z)
    pose.orientation.w = 1.0

    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def _make_oriented_box_collision_object(
    object_id: str,
    x_size: float,
    y_size: float,
    z_size: float,
    pos_x: float,
    pos_y: float,
    pos_z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
):
    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = str(object_id)

    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(x_size), float(y_size), float(z_size)]

    pose = Pose()
    pose.position.x = float(pos_x)
    pose.position.y = float(pos_y)
    pose.position.z = float(pos_z)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)

    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def _face_xy_unit_from_tag_pose(tag_pose: Pose) -> np.ndarray:
    _, _, tag_z = _tag_axes(tag_pose)
    face_xy = np.array([float(tag_z[0]), float(tag_z[1]), 0.0], dtype=float)
    face_norm = float(np.linalg.norm(face_xy))
    if face_norm < 1e-6:
        face_xy = np.array(
            [-float(tag_pose.position.x), -float(tag_pose.position.y), 0.0],
            dtype=float,
        )
        face_norm = float(np.linalg.norm(face_xy))
    if face_norm < 1e-6:
        return np.array([-1.0, 0.0, 0.0], dtype=float)

    face_xy = face_xy / face_norm
    to_robot = np.array(
        [-float(tag_pose.position.x), -float(tag_pose.position.y), 0.0],
        dtype=float,
    )
    to_robot_norm = float(np.linalg.norm(to_robot))
    if to_robot_norm > 1e-6:
        to_robot = to_robot / to_robot_norm
        if float(np.dot(face_xy, to_robot)) < 0.0:
            face_xy = -face_xy
    return face_xy


def _estimate_object_center_from_side_tag(tag_id: int, tag_pose: Pose) -> tuple[np.ndarray, str]:
    pose_xyz = np.array(
        [
            float(tag_pose.position.x),
            float(tag_pose.position.y),
            float(tag_pose.position.z),
        ],
        dtype=float,
    )
    obj = OBJECTS.get(int(tag_id))
    if obj is None or not hasattr(obj, "grasp_offset"):
        return pose_xyz, "pose"

    tag_x, tag_y, tag_z = _tag_axes(tag_pose)
    try:
        dx, dy, dz = obj.grasp_offset
        center_from_tag = (
            pose_xyz
            + (float(dx) * np.array(tag_x, dtype=float))
            + (float(dy) * np.array(tag_y, dtype=float))
            + (float(dz) * np.array(tag_z, dtype=float))
        )
        return center_from_tag, "tag_offset"
    except Exception:
        return pose_xyz, "pose"


def add_temporary_medication_face_keepout(
    node,
    *,
    tag_pose: Pose,
    keepout_id_prefix: str = "temp_med_face_keepout",
    tag_id: int = 1,
    rear_depth_m: float = 0.18,
    rear_width_m: float = 0.24,
    rear_height_m: float = 0.18,
    rear_center_back_offset_m: float = 0.02,
    side_center_use_horizontal_face_model: bool = True,
    side_face_to_center_m: float | None = None,
    side_tangent_offset_m: float = 0.0,
    top_cap_enable: bool = True,
    top_size_x_m: float = 0.14,
    top_size_y_m: float = 0.20,
    top_size_z_m: float = 0.05,
    top_clearance_m: float = 0.01,
) -> bool:
    pub = _get_scene_pub(node)
    front_xy = _face_xy_unit_from_tag_pose(tag_pose)
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    lateral = np.cross(up, front_xy)
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm < 1e-6:
        lateral = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        lateral = lateral / lateral_norm
    front_xy = front_xy / (float(np.linalg.norm(front_xy)) + 1e-9)
    rot = np.column_stack((front_xy, lateral, up))
    u, _, vt = np.linalg.svd(rot)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        rot[:, -1] *= -1
    rear_q = Rotation.from_matrix(rot).as_quat()

    base_height = float(TABLE_SURFACE_Z)
    obj_height_m = float(OBJECT_SHAPES.get(int(tag_id), {}).get("height", MEDICATION_HEIGHT))
    if bool(side_center_use_horizontal_face_model):
        default_face_to_center_m = float(
            OBJECT_SHAPES.get(int(tag_id), {}).get("radius", MEDICATION_RADIUS)
        )
        face_to_center_m = float(default_face_to_center_m) if side_face_to_center_m is None else max(
            0.0,
            float(side_face_to_center_m),
        )
        tangent_offset_m = float(side_tangent_offset_m)
        tangent_xy = np.array([-front_xy[1], front_xy[0], 0.0], dtype=float)
        center = np.array(
            [
                float(tag_pose.position.x) - (front_xy[0] * face_to_center_m) + (tangent_xy[0] * tangent_offset_m),
                float(tag_pose.position.y) - (front_xy[1] * face_to_center_m) + (tangent_xy[1] * tangent_offset_m),
                float(base_height + (0.5 * obj_height_m)),
            ],
            dtype=float,
        )
        center_source = (
            "scene_like_side_face"
            f"(face_to_center={face_to_center_m:.3f}, tangent={tangent_offset_m:.3f})"
        )
    else:
        center, center_source = _estimate_object_center_from_side_tag(int(tag_id), tag_pose)

    rear_depth = max(0.06, float(rear_depth_m))
    rear_width = max(0.08, float(rear_width_m))
    rear_height = max(0.06, float(rear_height_m))
    rear_back_offset = max(0.0, float(rear_center_back_offset_m))

    rear_center_xyz = (
        center
        - (front_xy * (0.5 * rear_depth + rear_back_offset))
    )
    rear_center_z = max(
        base_height + (0.5 * rear_height) + 0.005,
        float(center[2]),
    )
    rear_center_xyz[2] = rear_center_z

    rear_box = _make_oriented_box_collision_object(
        f"{keepout_id_prefix}_rear",
        rear_depth,
        rear_width,
        rear_height,
        float(rear_center_xyz[0]),
        float(rear_center_xyz[1]),
        float(rear_center_xyz[2]),
        float(rear_q[0]),
        float(rear_q[1]),
        float(rear_q[2]),
        float(rear_q[3]),
    )

    objects = [rear_box]

    if bool(top_cap_enable):
        top_size_x = max(0.05, float(top_size_x_m))
        top_size_y = max(0.05, float(top_size_y_m))
        top_size_z = max(0.02, float(top_size_z_m))
        top_gap = max(0.0, float(top_clearance_m))
        top_center_z = max(
            float(center[2]) + (0.5 * obj_height_m) + top_gap + (0.5 * top_size_z),
            base_height + (0.5 * top_size_z) + 0.01,
        )
        top_box = _make_box_collision_object(
            f"{keepout_id_prefix}_top",
            top_size_x,
            top_size_y,
            top_size_z,
            float(center[0]),
            float(center[1]),
            float(top_center_z),
        )
        objects.append(top_box)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = objects
    pub.publish(scene)
    node.get_logger().info(
        "Added temporary medication face keepout "
        f"'{keepout_id_prefix}' (rear_depth={rear_depth:.3f}m, rear_width={rear_width:.3f}m, "
        f"rear_height={rear_height:.3f}m, top_cap={'on' if bool(top_cap_enable) else 'off'}, "
        f"center_source={center_source})."
    )
    time.sleep(0.15)
    return True


def remove_temporary_medication_face_keepout(
    node,
    *,
    keepout_id_prefix: str = "temp_med_face_keepout",
) -> None:
    pub = _get_scene_pub(node)

    remove_ids = [
        f"{keepout_id_prefix}_rear",
        f"{keepout_id_prefix}_top",
    ]
    remove_objects = []
    for object_id in remove_ids:
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = "base_link"
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        remove_objects.append(co)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = remove_objects
    pub.publish(scene)
    node.get_logger().info(
        f"Removed temporary medication face keepout '{keepout_id_prefix}'."
    )
    time.sleep(0.10)


def add_temporary_table_guard_ring(
    node,
    *,
    guard_id_prefix: str = "temp_table_guard",
    margin_m: float = 0.5 * 0.0254,
    guard_height_m: float | None = None,
    wall_thickness_m: float = 0.5 * 0.0254,
) -> bool:
    pub = _get_scene_pub(node)

    margin = max(0.0, float(margin_m))
    wall_thickness = max(0.004, float(wall_thickness_m))
    if guard_height_m is None:
        # Keep the ring up to the tallest clear-table object so startup scans avoid grazing
        # unknown items around table edges without blocking overhead viewing.
        guard_height = max(
            float(BOTTLE_HEIGHT),
            float(MEDICATION_HEIGHT),
            float(CUP_HEIGHT),
            float(REMOTE_THICKNESS),
            float(CUBE_SIZE),
        )
    else:
        guard_height = max(0.03, float(guard_height_m))

    half_x = float(TABLE_X) / 2.0
    half_y = float(TABLE_Y) / 2.0
    center_x = float(TABLE_POS_X)
    center_y = float(TABLE_POS_Y)
    center_z = float(TABLE_SURFACE_Z) + 0.5 * guard_height

    span_x = float(TABLE_X) + (2.0 * margin) + (2.0 * wall_thickness)
    span_y = float(TABLE_Y) + (2.0 * margin) + (2.0 * wall_thickness)

    front_x = center_x - half_x - margin - (0.5 * wall_thickness)
    back_x = center_x + half_x + margin + (0.5 * wall_thickness)
    right_y = center_y - half_y - margin - (0.5 * wall_thickness)
    left_y = center_y + half_y + margin + (0.5 * wall_thickness)

    guard_objects = [
        _make_box_collision_object(
            f"{guard_id_prefix}_front",
            wall_thickness,
            span_y,
            guard_height,
            front_x,
            center_y,
            center_z,
        ),
        _make_box_collision_object(
            f"{guard_id_prefix}_back",
            wall_thickness,
            span_y,
            guard_height,
            back_x,
            center_y,
            center_z,
        ),
        _make_box_collision_object(
            f"{guard_id_prefix}_left",
            span_x,
            wall_thickness,
            guard_height,
            center_x,
            left_y,
            center_z,
        ),
        _make_box_collision_object(
            f"{guard_id_prefix}_right",
            span_x,
            wall_thickness,
            guard_height,
            center_x,
            right_y,
            center_z,
        ),
    ]

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = guard_objects
    pub.publish(scene)
    node.get_logger().info(
        "Added temporary table guard ring "
        f"'{guard_id_prefix}' (margin={margin:.4f}m, height={guard_height:.3f}m, thickness={wall_thickness:.4f}m)."
    )
    time.sleep(0.2)
    return True


def remove_temporary_table_guard_ring(node, *, guard_id_prefix: str = "temp_table_guard") -> None:
    pub = _get_scene_pub(node)

    remove_ids = [
        f"{guard_id_prefix}_front",
        f"{guard_id_prefix}_back",
        f"{guard_id_prefix}_left",
        f"{guard_id_prefix}_right",
    ]
    remove_objects = []
    for object_id in remove_ids:
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = "base_link"
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        remove_objects.append(co)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = remove_objects
    pub.publish(scene)
    node.get_logger().info(f"Removed temporary table guard ring '{guard_id_prefix}'.")
    time.sleep(0.1)


def add_temporary_table_top_keepout(
    node,
    *,
    keepout_id: str = "temp_table_top_keepout",
    margin_m: float = 0.5 * 0.0254,
    inset_x_m: float = 0.0,
    inset_y_m: float = 0.0,
    keepout_height_m: float = MEDICATION_HEIGHT + 0.02,
    base_z_m: float | None = None,
) -> bool:
    pub = _get_scene_pub(node)

    margin = max(0.0, float(margin_m))
    inset_x = max(0.0, float(inset_x_m))
    inset_y = max(0.0, float(inset_y_m))
    keepout_height = max(0.02, float(keepout_height_m))
    base_z = float(TABLE_SURFACE_Z) if base_z_m is None else float(base_z_m)
    center_z = base_z + (0.5 * keepout_height)

    size_x = max(0.05, float(TABLE_X) + (2.0 * margin) - (2.0 * inset_x))
    size_y = max(0.05, float(TABLE_Y) + (2.0 * margin) - (2.0 * inset_y))

    keepout = _make_box_collision_object(
        keepout_id,
        size_x,
        size_y,
        keepout_height,
        float(TABLE_POS_X),
        float(TABLE_POS_Y),
        center_z,
    )

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [keepout]
    pub.publish(scene)
    node.get_logger().info(
        "Added temporary table-top keepout "
        f"'{keepout_id}' (size=({size_x:.3f},{size_y:.3f},{keepout_height:.3f})m, "
        f"base_z={base_z:.3f}m, top_z={base_z + keepout_height:.3f}m, "
        f"inset_xy=({inset_x:.3f},{inset_y:.3f})m)."
    )
    time.sleep(0.15)
    return True


def remove_temporary_table_top_keepout(node, *, keepout_id: str = "temp_table_top_keepout") -> None:
    pub = _get_scene_pub(node)

    co = CollisionObject()
    co.header = Header()
    co.header.frame_id = "base_link"
    co.id = str(keepout_id)
    co.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [co]
    pub.publish(scene)
    node.get_logger().info(f"Removed temporary table-top keepout '{keepout_id}'.")
    time.sleep(0.1)
