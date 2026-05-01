# 2026-04-23 Latest Real-Arm Startup Run Notes

## Scope

These notes summarize the latest run logs found in `error-logs/` on this machine as of 2026-04-23.

Source logs:

- `error-logs/20260423_094956/T1_full.log`
- `error-logs/20260423_095109/T1_full.log`

No matching 2026-04-23 changelog existed in `changelogs/` before this file was created. The newest task-specific changelogs found were from 2026-04-21, with `OBJECT_PLACEMENT_EXPERIMENT_LOG.md` last modified on 2026-04-22.

## Runs Observed

| Run folder | Launch command | Controller update rate | Result |
|---|---|---:|---|
| `error-logs/20260423_094956` | `ros2 launch adl_tasks adl_start.launch.py arm_use_fake_hardware:=false arm_robot_ip:=192.168.0.10 launch_rviz:=true` | `1000 Hz` | Full stack started, but `KortexMultiInterfaceHardware` failed to connect to the robot. Controller manager then stayed in repeated initialization/wait states. |
| `error-logs/20260423_095109` | `ros2 launch adl_tasks adl_start.launch.py arm_use_fake_hardware:=false arm_robot_ip:=192.168.0.10 launch_rviz:=true` | `50 Hz` | Full stack started, but the same `KortexMultiInterfaceHardware: not connected !!!` failure occurred. Lowering update rate removed the repeated 1000 Hz overrun warnings, but did not solve the robot connection failure. |

## Main Finding

Both latest runs were blocked before task execution by real-hardware connection failure:

- `KortexMultiInterfaceHardware` attempted to connect to robot IP `192.168.0.10`.
- Hardware initialization failed with `not connected !!!`.
- `controller_manager` could not initialize the hardware component.
- `joint_state_broadcaster` could not contact `/controller_manager/list_controllers`.
- `scene_static` repeatedly reported that the planning scene service was unavailable.
- ADL task nodes reached their "ready/waiting for command" states, but real arm motion was not available because the hardware interface never initialized.

## Secondary Observations

- The wrist camera came up in both runs on camera source index `2` and published `1280x720` frames.
- AprilTag detection was receiving images, but no tags were detected during these startup windows.
- `pupil_apriltags` was not importable, so the AprilTag node used the OpenCV fallback backend.
- The UI server came up at `http://localhost:5000` and `http://192.168.0.231:5000`.
- Startup motion remained disabled; motion would begin only after an explicit UI/system task command.

## Difference Between The Two Runs

The first run used the ADL controller config at `1000 Hz` and logged controller-manager overrun warnings, for example missed 1000 Hz cycles.

The second run used the same real-arm launch command but the controller update rate was `50 Hz`. That run still failed on the same robot connectivity error, but the high-rate overrun warnings were no longer the dominant symptom.

Interpretation: the 50 Hz setting is likely better for reducing controller-loop timing noise on this machine, but the immediate blocker is network/session connectivity to the Kinova controller at `192.168.0.10`, not just controller update rate.

## Suggested Next Checks

Before rerunning ADL tasks on real hardware:

1. Confirm the Kinova controller is powered on, booted, and reachable at `192.168.0.10`.
2. Confirm the workstation network interface is on the robot subnet and has the expected wired connection.
3. Check whether another session/tool is already connected to the robot controller.
4. Keep the lower `50 Hz` controller update rate unless there is a specific reason to return to `1000 Hz`.
5. If AprilTag detection matters for the next run, decide whether the OpenCV fallback is acceptable or rebuild/run `adl_tasks` with a Python environment that has `pupil_apriltags` importable.

## New Concepts

- `KortexMultiInterfaceHardware`: the ROS 2 control hardware plugin that talks to the Kinova/Kortex arm. If it cannot connect, the task nodes can still start, but real arm controllers cannot become usable.
- `controller_manager`: the ROS 2 control process that loads and runs hardware interfaces and controllers. When hardware initialization fails, downstream controllers such as `joint_state_broadcaster` cannot be listed or started normally.
- `pupil_apriltags` fallback: this workspace can detect AprilTags through OpenCV when `pupil_apriltags` is missing, but the log warns about the missing module because the preferred/alternate detector is not available to the running Python interpreter.
