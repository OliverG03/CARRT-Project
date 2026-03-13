# spawn_sim_objs.py
# Spawn simple geometric shapes in Gazebo to represent vision_stub.py poses.
# Run AFTER Gazebo is launched and running.
#
# Usage:
#   ros2 run adl_tasks spawn_sim_objs
#
# Objects spawned match STUB_POSES in vision_stub.py exactly.
# Positions are in Gazebo world frame (base_link = world in sim).
 
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose
 
from adl_tasks.adl_config import (
    real_z,
    TABLE_SURFACE_Z, TABLE_POS_X, TABLE_POS_Y,
    BOTTLE_RADIUS,
    CUP_DIAMETER, CUP_HEIGHT,
    REMOTE_THICKNESS,
    CUBE_SIZE,
)
 
# --- SDF Templates ---
 
def cube_sdf(size: float, r: float, g: float, b: float) -> str:
    half = size / 2.0
    inertia = (1.0 / 6.0) * 0.2 * size ** 2  # solid cube inertia
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="cube">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>{inertia:.6f}</ixx>
          <iyy>{inertia:.6f}</iyy>
          <izz>{inertia:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1.0</ambient>
          <diffuse>{r} {g} {b} 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
 
def cylinder_sdf(radius: float, length: float,
                 r: float, g: float, b: float, a: float = 1.0,
                 mass: float = 0.3) -> str:
    ixx = (1.0/12.0) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="cylinder">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{ixx:.6f}</iyy>
          <izz>{izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
 
def box_sdf(x: float, y: float, z: float,
            r: float, g: float, b: float) -> str:
    mass = 0.1
    ixx = (1.0/12.0) * mass * (y**2 + z**2)
    iyy = (1.0/12.0) * mass * (x**2 + z**2)
    izz = (1.0/12.0) * mass * (x**2 + y**2)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="box">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{iyy:.6f}</iyy>
          <izz>{izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1.0</ambient>
          <diffuse>{r} {g} {b} 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
 
 
# --- Spawn Objects ---
# Each entry: (name, sdf_string, pose_x, pose_y, pose_z)
# Positions match STUB_POSES in vision_stub.py exactly
 
def make_pose(x, y, z) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.w = 1.0
    return p
 
SPAWN_OBJECTS = [
    # Cube (ID 4) — orange, on table, tag facing up
    {
        "name": "cube",
        "sdf": cube_sdf(CUBE_SIZE, 1.0, 0.4, 0.0),
        "pose": make_pose(
            TABLE_POS_X - 0.15,
            TABLE_POS_Y,
            TABLE_SURFACE_Z + CUBE_SIZE / 2.0   # sitting on table surface
        ),
    },
    # Cup (ID 2) — red, upright on table
    {
        "name": "cup",
        "sdf": cylinder_sdf(CUP_DIAMETER / 2.0, CUP_HEIGHT, 0.8, 0.1, 0.1),
        "pose": make_pose(
            TABLE_POS_X - 0.10,
            TABLE_POS_Y - 0.20,
            TABLE_SURFACE_Z + CUP_HEIGHT / 2.0
        ),
    },
    # TV Remote (ID 3) — grey, flat on table
    {
        "name": "remote",
        "sdf": box_sdf(0.043, 0.15, REMOTE_THICKNESS, 0.3, 0.3, 0.3),
        "pose": make_pose(
            TABLE_POS_X - 0.20,
            TABLE_POS_Y - 0.10,
            TABLE_SURFACE_Z + REMOTE_THICKNESS / 2.0
        ),
    },
    # Water Bottle (ID 0) — blue, on floor on its side
    {
        "name": "water_bottle",
        "sdf": cylinder_sdf(BOTTLE_RADIUS, 0.220, 0.2, 0.5, 1.0, 0.8, 0.5),
        "pose": make_pose(
            0.30,
            0.00,
            real_z(BOTTLE_RADIUS)   # resting on floor
        ),
    },
]
 
 
class SpawnSimObjectsNode(Node):
    def __init__(self):
        super().__init__('spawn_sim_objects_node')
        self.get_logger().info('Spawn Sim Objects Node started.')
 
        self._client = self.create_client(SpawnEntity, '/world/empty/create')
 
        self.get_logger().info('Waiting for Gazebo spawn service...')
        while not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo spawn service not available, waiting...')
 
        self.get_logger().info('Gazebo ready. Spawning objects...')
        self._spawn_all()
 
    def _spawn_all(self):
        spawned = 0
        for obj in SPAWN_OBJECTS:
            success = self._spawn(obj['name'], obj['sdf'], obj['pose'])
            if success:
                spawned += 1
                self.get_logger().info(
                    f"Spawned '{obj['name']}' at "
                    f"({obj['pose'].position.x:.3f}, "
                    f"{obj['pose'].position.y:.3f}, "
                    f"{obj['pose'].position.z:.3f})"
                )
            else:
                self.get_logger().error(f"Failed to spawn '{obj['name']}'.")
 
        self.get_logger().info(
            f'Done. Spawned {spawned}/{len(SPAWN_OBJECTS)} objects.'
        )
 
    def _spawn(self, name: str, sdf: str, pose: Pose) -> bool:
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose = pose
 
        future = self._client.call_async(req)
        start = self.get_clock().now()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().error(f"Spawn request for '{name}' timed out.")
                return False
 
        result = future.result()
        if result is None:
            return False
        if not result.success:
            self.get_logger().error(f"Spawn failed for '{name}': {result.status_message}")
            return False
        return True
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SpawnSimObjectsNode()
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()