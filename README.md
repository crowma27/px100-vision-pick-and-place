# PX100 Vision Pick-and-Place

A **monocular-vision pick-and-place** system for the PX100 robot arm in CoppeliaSim.
The arm picks up a blue cube and drops it on a green target, using **only one RGB camera** — no depth sensor.

## Core idea

> A pixel is a **direction**, not a position.
> The table is a **known plane** (`z = 0`).
> Where the direction intersects the plane — **that is the 3D point**.

This is why a monocular camera is enough: the table does the job a depth sensor would normally do.

## Architecture


┌──────────────┐ HSV ┌───────────────┐
│ RGB image │─────────▶│ Colour mask │
│ 640 × 480 │ │ + centroid │
└──────────────┘ └───────┬───────┘
│ (u, v)
▼
┌──────────────────┐
│ CameraModel │
│ pixel_to_world() │
└────────┬─────────┘
│ (x, y, z)
▼
┌──────────────────┐
│ FSM_Machine │
│ APPROACH → ... │
└────────┬─────────┘
│ q_cmd
▼
┌──────────────────┐

│ arm_interface │
│ set_joints() │
└──────────────────┘



## File layout


src/
main_program.m Orchestrator
config_px100.m All parameters
simulator_interface.m CoppeliaSim connection
arm_interface.m PX100 arm connection
calculate_IK_px100.m Inverse kinematics
FSM_Machine.m State machine
JointUtils.m Clamp + tolerances
LiveView.m Live video window
VisionDetector.m HSV colour detection
CameraModel.m Pixel → world
capture_scene.m One photo → (x, y, z)
docs/
VISION_THEORY.md Full theory with derivations



## Requirements

- **MATLAB** R2021a or newer (uses `arguments` and `isfile`)
- **CoppeliaSim** 4.5+ with the PX100 scene
- **JeroMQ** (`jeromq-0.5.2.jar` — auto-downloaded by `RemoteAPIClient`)

## How to run

1. Start CoppeliaSim with the PX100 scene.
2. In MATLAB, navigate to `src/`:
   ```matlab
   cd src


How it works (summary)

    The arm retracts to q_home so it does not block the camera.

    capture_scene takes one photo, detects blue and green, back-projects both.

    The FSM runs the cycle: APPROACH → GRASP → LIFT → DROP → RELEASE → HOME.

    Vision runs once — the control loop is blind to vision afterwards.

Main formula (vertical camera)


x = Cx + h · (u − cx) / f
y = Cy − h · (v − cy) / f
z = 0

