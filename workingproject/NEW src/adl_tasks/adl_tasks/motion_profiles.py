# ------ motion_profiles.py ------ #
# Hold information on tolerances and motion-tuning. 
# Created: PoseTolerance and MotionProfile dataclasses to hold this information, improving on messy files before in clear_table/helper_moves

from dataclasses import dataclass

@dataclass(frozen=True)
class PoseTolerance:
    pos: float = 0.04       # m
    ori_xy: float = 0.6     # rads
    ori_z: float = 3.14

@dataclass(frozen=True)
class MotionProfile:
    planning_time: float = 10.0
    velocity_scaling: float = 0.3
    accel_scaling: float = 0.3
    
# default profiles

DEFAULT_PROFILE = MotionProfile()
APPROACH_TOL = PoseTolerance(pos=0.03, ori_xy=0.4, ori_z=3.14)
DROP_ALIGN_TOL = PoseTolerance(pos=0.06, ori_xy=0.6, ori_z=3.14)
DROP_ALIGN_TIGHT_TOL = PoseTolerance(pos=0.06, ori_xy=0.35, ori_z=3.14)