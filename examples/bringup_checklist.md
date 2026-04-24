# StudentLib Real-Robot Bring-Up Checklist

## Before power-on / before first motion

- [ ] Put the robot on blocks so wheels can spin freely.
- [ ] Confirm motor ports match the code.
- [ ] Confirm left and right motor groups are not swapped.
- [ ] Confirm all sensor ports match the code.
- [ ] Confirm all distances are in inches.
- [ ] Confirm all angles in code are radians, not degrees.

## Verify motor directions

- [ ] Run a small manual tank command: left = 0.2, right = 0.2.
- [ ] Expected: both sides drive forward.
- [ ] If one side spins backward, reverse that motor port or motor direction.
- [ ] Run a small turn command: left = -0.2, right = 0.2.
- [ ] Expected: robot turns counterclockwise if positive heading is CCW.

## Verify IMU heading direction

- [ ] Reset pose to `{0, 0, 0}`.
- [ ] Rotate robot counterclockwise by hand.
- [ ] Expected: heading increases.
- [ ] Rotate robot clockwise by hand.
- [ ] Expected: heading decreases.
- [ ] If heading sign is wrong, check IMU mounting/yaw offset/sign conventions.

## Verify tracker wheel sign conventions

- [ ] Reset pose to `{0, 0, 0}`.
- [ ] Push robot forward by hand.
- [ ] Expected: x increases if heading is `0`.
- [ ] Push robot backward by hand.
- [ ] Expected: x decreases.
- [ ] If available, slide robot left/right by hand.
- [ ] Expected: lateral tracker changes the expected field axis.
- [ ] If forward motion produces negative displacement, reverse the vertical tracker sensor.
- [ ] If x/y appear swapped, check tracker axis configuration.

## Verify pose reset

- [ ] Move/rotate robot by hand and confirm pose changes.
- [ ] Call `setPose({0, 0, 0})`.
- [ ] Expected: printed pose returns near zero.
- [ ] Call `resetPose()`.
- [ ] Expected: estimator resets to zero pose and re-baselines on the next update.

## Run localization-only tests first

- [ ] Run `validate_localization_basic.cpp`.
- [ ] Do not command autonomous motion yet.
- [ ] Move the robot by hand and watch x, y, theta.
- [ ] Fix sign/axis issues before tuning motion.

## Run motion smoke tests second

- [ ] Start with short distances: 6 to 12 inches.
- [ ] Use low max speed: 0.3 to 0.5.
- [ ] Test turning before long driving.
- [ ] Test one motion at a time.
- [ ] Print pose and motion status after each command.

## Verify timeouts and cancel behavior

- [ ] Start a nonblocking motion.
- [ ] Cancel it after a short delay.
- [ ] Expected status: `CANCELED`.
- [ ] Use a deliberately short timeout.
- [ ] Expected status: `TIMED_OUT`.
- [ ] If commands never settle, loosen tolerances or check localization noise.

## Verify each MoveToPose path mode

- [ ] `POSE_REGULATOR`: should move directly toward the target pose.
- [ ] `STRAIGHT_THEN_TURN`: should drive to position, then rotate to final heading.
- [ ] `QUADRATIC_BEZIER`: should follow a smoother curved approach.
- [ ] Use the same small target for all modes first.
- [ ] Increase speed only after signs and settling are correct.

## Common symptoms and likely causes

- Robot turns opposite expected direction:
  - Left/right motor groups swapped.
  - Turn sign convention mismatch.
  - IMU heading sign mismatch.

- Forward motion produces negative displacement:
  - Vertical tracker reversed.
  - Drive motor direction reversed.
  - Pose heading initialized incorrectly.

- x and y appear swapped:
  - Tracker axis configured incorrectly.
  - Robot-frame sensor mounting assumptions wrong.

- IMU heading increases in the wrong direction:
  - IMU orientation/sign convention mismatch.
  - Yaw offset/sign needs correction.

- Motion overshoots badly:
  - kP too high.
  - kD too low.
  - maxSpeed too high.
  - Localization is noisy or delayed.

- Command never settles:
  - Tolerances too tight.
  - Velocity tolerance too strict.
  - Robot physically oscillates near target.
  - Pose estimate is drifting.

- Command times out unexpectedly:
  - timeoutSeconds too short.
  - maxSpeed too low.
  - minSpeed too low to overcome friction.
  - Target is unreachable or localization is wrong.