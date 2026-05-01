# ------ motion_profiles.py ------ #
# Hold information on tolerances and motion-tuning. 
# Keeps motion tolerances and profile defaults shared across tasks.

from dataclasses import dataclass

@dataclass(frozen=True)
class PoseTolerance:
    pos: float = 0.04       # m
    ori_xy: float = 0.6     # rads
    ori_z: float = 3.14

@dataclass(frozen=True)
class MotionProfile:
    planning_time: float = 10.0
    # Match the validated horizontal-scan profile for general moves so fixed and pose-goal
    # transitions run at a consistent, field-tested speed envelope.
    velocity_scaling: float = 0.25
    accel_scaling: float = 0.25
    
# default profiles

DEFAULT_PROFILE = MotionProfile()
APPROACH_TOL = PoseTolerance(pos=0.03, ori_xy=0.4, ori_z=3.14)
DROP_ALIGN_TOL = PoseTolerance(pos=0.06, ori_xy=0.6, ori_z=3.14)
DROP_ALIGN_TIGHT_TOL = PoseTolerance(pos=0.06, ori_xy=0.35, ori_z=3.14)
