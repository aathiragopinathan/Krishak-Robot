# Camera and robot integration

## Coordinate contract

Image coordinates are `(u, v)` in pixels, with the origin at the top left. Robot coordinates are `(x, y, z)` in metres: x points into the working patch, y points left, and z points upward. The example ground is z = 0 in the arm-base frame. Joint angles and wheel speeds use radians and radians per second.

`GroundMapper` applies a homography:

```text
[a, b, c]ᵀ = H [u, v, 1]ᵀ
x = a / c
y = b / c
z = 0
```

For a fixed camera and approximately planar soil, measure at least four non-collinear ground points in the arm-base frame and their corresponding image points. Fit the homography, then check it against independent points across the full working patch. Correct lens distortion before fitting and before inference. A changed image resolution, crop, camera mount, or ground plane invalidates the calibration. Elevated leaves introduce parallax; a depth sensor or multi-view geometry is a useful extension.

Example JSON for the **640×480 generated scene only**:

```json
[[0, -0.00035, 0.37], [0.0005, 0, -0.16], [0, 0, 1]]
```

The working patch is x ∈ [0.15, 0.38] m and y ∈ [-0.18, 0.18] m. This is a software selection region, not a collision-free workspace guarantee. The target centroid can differ from the stem or treatment point.

## Robot adapter boundary

`SimRobot` provides `drive`, `stop`, `move_arm`, `set_gripper`, and `record_uproot`. It models commands as immediate successes. The scheduler approaches, grasps, lifts, moves to an example basket location, releases, and returns home. A physical adapter must instead expose command completion, measured wheel stop, measured arm pose, grip/removal confirmation, faults, and cancellation. The FSM would need waiting states and deadlines for each asynchronous operation.

Before using a camera observation for targeting, stop the platform, confirm settling, capture with a timestamp, and associate the camera transform with that capture. Do not reuse targets after the rover or camera has moved. The current demo is one stationary observation, with no timestamp or odometry model.

The six wheel speeds assume three wheels per side with equal speed on each side, 0.4 m track width, and 0.08 m wheel radius. These values are examples. The paper proposes rocker-bogie suspension and lists 65 mm wheels, but does not provide measured track width or sufficient details to reproduce its wheel controller. The simulator does not model the suspension or the four-wheel SEED chassis.

The arm's standard DH chain is stored as `(a, alpha, d)` with six revolute joint variables. Its numerical IK solves position only and may fail at singularities or from poor initial guesses. A converged endpoint does not establish a safe path. Physical integration needs measured joint limits, encoder calibration, velocity/acceleration constraints, self-collision and environment models, and an appropriate end-effector orientation.

## Bring-up sequence

1. Validate image masks and ground projection using a fixed calibration board.
2. Check measured arm geometry using forward kinematics at known poses.
3. Run trajectories without a tool or agricultural action and compare commanded and measured positions.
4. Verify drive/arm interlocks, communication timeouts, and a physical emergency stop.
5. Test individual targets under supervision before repeated field sequences.

The paper identifies mechanical uprooting and basket collection, and shows a gripper on the SEED prototype. Finger geometry, grip force, extraction force, and removal confirmation remain unspecified. `record_uproot` logs a simulated sequence completion, not evidence of physical removal. A fault stops the rover and suppresses further commands; it does not automatically open a loaded gripper or provide physical arm braking.

## Paper-specific interfaces

The manuscript describes HuskyLens-to-Arduino serial communication and Raspberry Pi planning. A future adapter should translate validated, timestamped camera detections into the same ground-coordinate targets used here. Do not send image pixels directly to inverse kinematics. An arm-mounted camera changes its transform as the joints move: either capture at a repeatable known pose or use joint feedback with hand-eye calibration.

AprilTag localization and encoder/AHRS feedback belong to the navigation layer. They need camera intrinsics, known tag geometry and placement, frame transforms, synchronization, and feedback controllers. None are replaced by the stationary ground homography. See [paper notes](paper-notes.md) for the documented architecture and ambiguous component identifiers.
