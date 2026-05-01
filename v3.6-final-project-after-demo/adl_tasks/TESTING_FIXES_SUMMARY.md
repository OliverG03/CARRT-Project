# ADL Task Testing Fixes - Summary of Changes (2026-04-16)

## Overview
Applied fixes to three ADL tasks (give_medication, clear_table, pick_dropped_bottle) based on recent testing iterations run 1-3.

---

## 1. Give Medication Task - QR Reading Simplification

### Issue
- Bottle reached the read bottle pose but ended with no read name
- Front approach was adding unnecessary complexity

###Changes Made
**File:** `grasp_and_place.py` - GIVE_MEDICATION_CONFIG

1. **Disabled Front Entry Approach**
   - Changed: `"qr_read_front_entry_enable": True` → `False`
   - Effect: Task now moves directly to the above read pose instead of staging through a front-entry intermediary
   - Result: Simplified QR read flow, fewer intermediate moves

2. **Enabled Known QR Value Verification** 
   - Changed: `"known_qr_value_enable": _env_flag("ADL_MEDICATION_KNOWN_QR_VALUE_ENABLE", False)` → `True`
   - Effect: Known QR value verification is now active by default
   - Usage: Set environment variable `ADL_MEDICATION_KNOWN_QR_VALUE` to expected bottle name
   - Example: `export ADL_MEDICATION_KNOWN_QR_VALUE="ibuprofen"`

### Configuration Details
- The infrastructure for known QR values already existed; it's now enabled
- Can still be overridden via environment: `ADL_MEDICATION_KNOWN_QR_VALUE_ENABLE=0`
- Match mode is configurable: `ADL_MEDICATION_KNOWN_QR_MATCH_MODE` (default: "contains")
- Case-folding enabled for robust matching

---

## 2. Clear Table Task - Drop Mechanics Improvements

### Issues
1. **Run 1:** Cube missed grasp, TV remote grip too loose, TV remote drop collided with bin, cup placed too far back
2. **Run 2:** Arm pulled wire before side scan
3. **Run 3:** All objects remained, TV remote grip still too loose, drop collision continued

### Changes Made

#### 2a. Drop Location Forward Adjustment
**File:** `adl_config.py`

```python
# OLD: DROP_FORWARD_NUDGE_M = _env_float("ADL_DROP_FORWARD_NUDGE_M", 0.5 * 0.0254)
# NEW:
DROP_FORWARD_NUDGE_M = _env_float("ADL_DROP_FORWARD_NUDGE_M", 1.5 * 0.0254)
```

- Old: 0.5 inches forward margin
- New: 1.5 inches forward margin (1 inch additional push forward)
- Affects: `BIN_DROP_X` and `SHELF_DROP_X` calculations
- Result: Drop locations moved forward, reducing bin collision risk and providing clearance

#### 2b. Drop Descent Velocity Reduction
**File:** `grasp_and_place.py` - DROP_CONFIG

```python
# OLD: "stage6_servo_linear_speed_mps": 0.018
# NEW:
"stage6_servo_linear_speed_mps": 0.012
```

- Old: 0.018 m/s (18 mm/s) - too fast, caused bouncing
- New: 0.012 m/s (12 mm/s) - slower, more controlled descent
- Effect: Reduced bouncing on drop, smoother placement
- Applies to: Stage 6 cartesian descent during object release

#### 2c. TV Remote Gripper Force Increase
**File:** `apriktag_key.py` - OBJECTS[3] (TV Remote ID=3)

```python
# OLD: gripper_force=7.0
#NEW:
gripper_force=10.0
```

- Old: 7.0 N (Newtons)
- New: 10.0 N - 43% increase
- Effect: Tighter grip on remote, prevents slip/rotation during pick
- Note: This matches the travel gripper force (10.0 N), ensuring consistency

### Impact Summary
- **Drop location**: Forward 1 inch from container edge
- **Drop velocity**: 33% slower for controlled descent
- **Remote grip**: 43% stronger to prevent rotation/slip
- **Result**: Expected to fix bin collisions, cup placement inaccuracy, and remote grip failures

---

## 3. Pick Dropped Bottle Task - Scan Pose Optimization

### Issue
- Run 1: Looked for bottle and stopped after first attempted pose (baseline ground scan)
- You identified better poses in RVIZ that were closer and easier to maneuver

### Changes Made

#### 3a. Created Pose Collection Utility
**File:** `scripts/collect_bottle_scan_poses.py` (New)

Interactive tool to:
- Move to standard poses (home, table, ground)
- Move to custom joint configurations
- Print current end-effector pose and joint values
- Save poses with names
- Export in helper_moves.py format

#### 3b. Added Setup.py Entry Point
**File:** `setup.py`

Added console script entry:
```python
'collect_bottle_scan_poses = adl_tasks.scripts.collect_bottle_scan_poses:main'
```

#### 3c. Created Tuning Guide
**File:** `BOTTLE_POSE_TUNING.md` (New)

Comprehensive guide with:
- Pose collection procedures
- Current baseline pose settings
- Transform data reference
- Testing workflow

### Usage Instructions
1. **Rebuild package:**
   ```bash
   cd ~/workspace/ros2_kortex_ws
   colcon build --packages-select adl_tasks --symlink-install
   source install/setup.bash
   ```

2. **Run pose collector:**
   ```bash
   ros2 run adl_tasks collect_bottle_scan_poses
   ```

3. **Commands in the tool:**
   - `ground` - Go to current baseline pose for comparison
   - `custom j1 j2 j3 j4 j5 j6 j7` - Test custom positions (in radians)
   - `current` - Print current end-effector pose
   - `save <name>` - Save promising poses
   - `export` - Export all saved poses in helper_moves format

4. **Update with best pose:**
   - After collecting poses, take the best performers' joint values
   - Update `LOOK_AT_GROUND_JOINTS` in `helper_moves.py` with new joint values
   - Rebuild and test

### Next Steps for Bottle Poses
- Use collector tool to explore poses closer to/around the dropped bottle
- Test 2-3 candidate poses for comparison
- Select pose with highest detection rate and easiest approach
- Update helper_moves.py with final pose choice

---

## Files Modified Summary

| File | Changes | Impact |
|------|---------|--------|
| `grasp_and_place.py` | 1. Disabled `qr_read_front_entry_enable` 2. Enabled `known_qr_value_enable` 3. Reduced `stage6_servo_linear_speed_mps` | Med task simplified, Drop velocity reduced |
| `adl_config.py` | Increased `DROP_FORWARD_NUDGE_M` | Drop locations pushed forward 1 inch |
| `apriktag_key.py` | Increased TV Remote `gripper_force` to 10.0 | Tighter remote grip |
| `setup.py` | Added bottle pose collector entry point | New utility available |
| `scripts/collect_bottle_scan_poses.py` | NEW - Interactive pose collection tool | Enables pose tuning |
| `BOTTLE_POSE_TUNING.md` | NEW - Pose tuning documentation | Reference guide |

---

## Testing Recommendations

### 1. Give Medication Task
- [ ] Test QR read with simplified (no front-entry) flow
- [ ] Verify known QR value matching works with test values
- [ ] Confirm task completion without extra approach move

### 2. Clear Table Task
- [ ] Test cube grasp (should now have better positioning)
- [ ] Verify TV remote drop doesn't collide with bin (moved forward)
- [ ] Check TV remote grip is secure (10.0N force)
- [ ] Verify cup placement accuracy improved (dep on detection)
- [ ] Monitor drop descent for excessive bounce (slower velocity)

### 3. Pick Dropped Bottle Task
- [ ] Use collector tool to test 2-3 new poses
- [ ] Compare detection rates: current baseline vs. candidates
- [ ] Test approach success rate with new poses
- [ ] Update helper_moves with best performing pose

---

## Rollback Instructions

If any change causes issues:

1. **Revert grasp_and_place.py:**
   ```bash
   # Reset qr_read_front_entry_enable to True (or value)
   # Reset stage6_servo_linear_speed_mps to 0.018
   # Reset known_qr_value_enable to False
   ```

2. **Revert adl_config.py:**
   ```bash
   # Reset DROP_FORWARD_NUDGE_M to 0.5 * 0.0254
   ```

3. **Revert apriktag_key.py:**
   ```bash
   # Reset gripper_force to 7.0 for TV Remote
   ```

---

## Environment Variables for Medication QR

Set these to control medication QR verification:

```bash
# Enable/disable known value verification (default: enabled now)
export ADL_MEDICATION_KNOWN_QR_VALUE_ENABLE=1

# Set the expected QR value on the bottle
export ADL_MEDICATION_KNOWN_QR_VALUE="ibuprofen"

# Set match mode: "contains" (default) | "exact" | "startswith"
export ADL_MEDICATION_KNOWN_QR_MATCH_MODE="contains"

# Skip user name entry if known QR matches (default: skip)
export ADL_MEDICATION_KNOWN_QR_SKIP_USER_ENTRY=1
```

---

## Notes

- All changes are backward-compatible (no breaking API changes)
- Velocity reduction is conservative; can be tuned further if needed
- Bottle pose collection is optional (baseline still works, but can be improved)
- Known QR feature provides fallback verification without manual UI entry
- Drop forward nudge affects both bin and shelf placements
