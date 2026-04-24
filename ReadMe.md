# StudentLib PROS Robotics Library

StudentLib is a student-readable C++ robotics library for PROS-based VEX-style differential/tank-drive robots.

The library is designed as a clear, teachable alternative to larger robotics libraries. It emphasizes explicit subsystem boundaries, readable math, and practical autonomous motion commands.

## Project Goals

StudentLib is intended to be:

- PROS-based
- readable for students learning robotics software
- explicit rather than clever
- focused on differential/tank-drive robots in v1
- capable of odometry by default
- capable of optional MCL localization
- usable for autonomous routines
- customizable through PID, settling, and motion parameters
- free of hidden global robot state
- layered with one-directional dependencies

Typical high-level usage:

```cpp
robot.setPose({0.0, 0.0, 0.0});

robot.moveToPose({24.0, 12.0, 1.57});
robot.turnToPoint({48.0, 24.0});
robot.waitUntilSettled();
````

Nonblocking usage is also supported:

```cpp
robot.startMoveToPose({36.0, 0.0, 0.0});

// run intake, arm, piston, etc. while chassis is moving

while (!robot.isMotionComplete()) {
    pros::delay(10);
}
```

---

# Units and Coordinate Convention

The library uses one internal convention.

## Units

| Quantity                  | Internal Unit         |
| ------------------------- | --------------------- |
| Distance                  | inches                |
| Angle                     | radians               |
| Time                      | seconds               |
| Drivetrain voltage        | millivolts            |
| Normalized drive commands | usually `[-1.0, 1.0]` |

Degree helpers exist, but internal math should remain radians.

## Robot Frame

```text
+x     forward
+y     left
+theta counterclockwise
```

## Field Frame

The same convention is used in the field/world frame:

```text
+x     chosen forward/rightward field axis
+y     chosen left/upward field axis
+theta counterclockwise
```

---

# Architecture

The dependency direction is:

```text
math
-> hardware
-> model / drivetrain / field
-> estimation
-> control
-> motion
-> robot
-> api
```

Higher layers may depend on lower layers. Lower layers should not depend on higher layers.

Key rules:

* `Robot` is a facade and composition root, not a god class.
* `StateEstimator` is the localization abstraction boundary.
* Motion commands depend on `StateEstimator`, not on `OdometryEstimator` or `MCLEstimator` directly.
* `Drivetrain` does not know about localization.
* `PIDController` does not know about poses, drivetrains, or robots.
* `MotionManager` owns one active command at a time.
* MCL is included but optional.
* No singletons.
* No hidden globals inside the library.

---

# Recommended Project Layout

A clean PROS project using this library can look like:

```text
src/
├── main.cpp
├── support.hpp
├── support.cpp
└── studentlib source files...

include/
└── studentlib/
    └── library headers...
```

Recommended responsibility split:

| File             | Responsibility                                                           |
| ---------------- | ------------------------------------------------------------------------ |
| `main.cpp`       | PROS lifecycle hooks, autonomous sequence, driver control                |
| `support.hpp`    | exposes `support::robot()`, `support::initializeRobot()`, small helpers  |
| `support.cpp`    | ports, geometry, sensor config, robot construction, joystick/LCD helpers |
| `studentlib/...` | reusable robotics library                                                |

This keeps `main.cpp` simple.

---

# File Overview

## Core Files

### `include/studentlib/core/types.hpp`

Defines small core shared types.

Current contents include:

* `timestamp_ms_t`
* `AxisType`

### `include/studentlib/core/units.hpp`

Defines simple unit helpers and constants.

Examples:

```cpp
feetToInches(...)
inchesToFeet(...)
metersToInches(...)
secondsToMilliseconds(...)
voltsToMillivolts(...)
```

### `include/studentlib/core/result.hpp`

Defines a lightweight `Result` type for success/error status.

### `include/studentlib/core/time_util.hpp`

Wraps basic PROS timing:

```cpp
nowMilliseconds()
nowSeconds()
delayMilliseconds(...)
delaySeconds(...)
hasElapsed(...)
```

---

## Math Files

### `include/studentlib/math/point.hpp`

Defines:

```cpp
struct Point2D {
    double x;
    double y;
};
```

Used for field/world positions and target points.

### `include/studentlib/math/vector.hpp`

Defines:

```cpp
struct Vector2D {
    double x;
    double y;
};
```

Includes basic vector operators.

### `include/studentlib/math/pose.hpp`

Defines:

```cpp
struct Pose2D {
    double x;
    double y;
    double theta;
};
```

Represents position plus heading.

### `include/studentlib/math/twist.hpp`

Defines:

```cpp
struct Twist2D {
    double vx;
    double vy;
    double omega;
};
```

Represents translational and angular velocity.

### `include/studentlib/math/angle.hpp`

Defines angle helpers:

```cpp
degreesToRadians(...)
radiansToDegrees(...)
normalizeAngle(...)
normalizeAnglePositive(...)
smallestAngleDifference(...)
anglesNear(...)
```

`normalizeAngle()` returns angles in approximately `[-pi, pi)`.

### `include/studentlib/math/math_util.hpp`

Defines general math helpers:

```cpp
clamp(...)
sign(...)
square(...)
lerp(...)
dot(...)
cross(...)
distance(...)
headingToPoint(...)
headingVector(...)
rotateVector(...)
transformPoint(...)
inverseTransformPoint(...)
```

These are used by odometry, MCL, field maps, and motion.

---

# Hardware Layer

The hardware layer wraps PROS devices immediately. Higher layers should not pass around raw PROS devices.

## `MotorGroup`

File:

```text
include/studentlib/hardware/motor_group.hpp
src/hardware/motor_group.cpp
```

Responsibilities:

* owns one or more `pros::Motor`
* supports motor reversal
* sets brake mode
* commands voltage
* commands velocity
* stops motors
* returns average position and velocity
* tares motor positions

Main API:

```cpp
MotorGroup(const std::vector<int>& motorPorts);
MotorGroup(const std::vector<int>& motorPorts,
           const std::vector<bool>& reversedFlags);

void setBrakeMode(pros::motor_brake_mode_e_t mode);
void setVoltage(double millivolts);
void setVelocity(double rpm);
void stop();

void tarePosition();

double getAveragePositionRadians() const;
double getAverageVelocityRadPerSec() const;
std::size_t size() const;
```

Negative motor ports are treated as reversed motors.

Example:

```cpp
MotorGroup left({1, 2, 3});
MotorGroup right({-4, -5, -6});
```

---

## `IMUSensor`

File:

```text
include/studentlib/hardware/imu_sensor.hpp
src/hardware/imu_sensor.cpp
```

Wraps `pros::Imu`.

Main API:

```cpp
explicit IMUSensor(std::int8_t port, double yawOffsetRadians = 0.0);

void reset();
bool isCalibrating() const;

double getHeadingRadians() const;
double getRawHeadingRadians() const;

void setYawOffset(double yawOffsetRadians);
double getYawOffset() const;
```

Expected convention:

* counterclockwise rotation should increase heading
* heading is returned in radians

---

## `RotationSensor`

File:

```text
include/studentlib/hardware/rotation_sensor.hpp
src/hardware/rotation_sensor.cpp
```

Wraps `pros::Rotation`.

Main API:

```cpp
explicit RotationSensor(std::int8_t port, bool reversed = false);

void reset();

double getAngleRadians() const;
double getDeltaRadians();

bool isReversed() const;
```

Used by tracker wheels.

---

## `DistanceSensor`

File:

```text
include/studentlib/hardware/distance_sensor.hpp
src/hardware/distance_sensor.cpp
```

Wraps `pros::Distance`.

Main API:

```cpp
explicit DistanceSensor(std::uint8_t port);

std::optional<double> getDistanceInches() const;
```

Invalid readings are returned as `std::nullopt`.

---

## `TrackerWheel`

File:

```text
include/studentlib/hardware/tracker_wheel.hpp
src/hardware/tracker_wheel.cpp
```

Composes a `RotationSensor` and converts wheel rotation to travel distance.

Main API:

```cpp
TrackerWheel(RotationSensor rotationSensor,
             double wheelRadiusInches,
             double gearRatio,
             TrackerAxis axis);

void reset();

double getDistanceTraveled() const;
double getDeltaDistance();

double getWheelRadius() const;
double getGearRatio() const;
TrackerAxis getAxis() const;
```

Tracker axis:

```cpp
enum class TrackerAxis {
    LONGITUDINAL,
    LATERAL
};
```

Meaning:

| Axis           | Measures                |
| -------------- | ----------------------- |
| `LONGITUDINAL` | forward/backward travel |
| `LATERAL`      | sideways travel         |

Mounting geometry is handled by the model layer, not by `TrackerWheel`.

---

## `SensorSuite`

File:

```text
include/studentlib/hardware/sensor_suite.hpp
src/hardware/sensor_suite.cpp
```

Owns optional hardware sensors.

Main API:

```cpp
void setIMU(IMUSensor imu);
bool hasIMU() const;

IMUSensor* getIMU();
const IMUSensor* getIMU() const;

void addTrackerWheel(TrackerWheel tracker);
std::vector<TrackerWheel>& getTrackerWheels();
const std::vector<TrackerWheel>& getTrackerWheels() const;

void addDistanceSensor(DistanceSensor sensor);
std::vector<DistanceSensor>& getDistanceSensors();
const std::vector<DistanceSensor>& getDistanceSensors() const;

void reset();
```

---

# Model and Field Layer

## `DrivetrainConfig`

File:

```text
include/studentlib/model/drivetrain_config.hpp
src/model/drivetrain_config.cpp
```

Physical drivetrain constants:

```cpp
struct DrivetrainConfig {
    double trackWidthInches;
    double wheelRadiusInches;
    double gearRatio;
    double maxVoltageMillivolts;

    bool isValid() const;
};
```

---

## `sensor_mounts.hpp`

Defines mounting metadata.

Types:

```cpp
struct MountedIMU {
    IMUSensor* imu;
    double yawOffsetRadians;
};

struct MountedTrackerWheel {
    TrackerWheel* tracker;
    Point2D positionInRobotFrame;
};

struct MountedDistanceSensor {
    DistanceSensor* sensor;
    Pose2D poseInRobotFrame;
    double maxRangeInches;
    double beamAngleOffsetRadians;
};
```

These are non-owning references. Hardware ownership lives in `SensorSuite`.

---

## `RobotModel`

File:

```text
include/studentlib/model/robot_model.hpp
src/model/robot_model.cpp
```

Stores physical robot geometry and sensor mounting metadata.

Main API:

```cpp
RobotModel();
explicit RobotModel(const DrivetrainConfig& drivetrainConfig);

void setDrivetrainConfig(const DrivetrainConfig&);
const DrivetrainConfig& getDrivetrainConfig() const;

void setMountedIMU(const MountedIMU&);
bool hasMountedIMU() const;
const std::optional<MountedIMU>& getMountedIMU() const;

void addMountedTrackerWheel(const MountedTrackerWheel&);
const std::vector<MountedTrackerWheel>& getMountedTrackerWheels() const;

void addMountedDistanceSensor(const MountedDistanceSensor&);
const std::vector<MountedDistanceSensor>& getMountedDistanceSensors() const;
```

---

## `FieldMap`

File:

```text
include/studentlib/model/field_map.hpp
src/model/field_map.cpp
```

Simple 2D field representation for MCL.

Types:

```cpp
struct LineSegment2D {
    Point2D start;
    Point2D end;
};

struct FieldBounds {
    double minX;
    double maxX;
    double minY;
    double maxY;
};
```

Main API:

```cpp
void addSegment(const LineSegment2D& segment);
const std::vector<LineSegment2D>& getSegments() const;

static FieldMap makeRectangularField(double widthInches, double heightInches);

std::optional<double> raycastDistance(
    const Point2D& origin,
    double directionRadians,
    double maxDistanceInches) const;

std::optional<FieldBounds> getBounds() const;
```

Used by MCL distance-sensor prediction.

---

# Drivetrain Layer

## `Drivetrain`

File:

```text
include/studentlib/drivetrain/drivetrain.hpp
```

Abstract drivetrain interface.

```cpp
class Drivetrain {
public:
    virtual void tank(double left, double right) = 0;
    virtual void arcade(double forward, double turn) = 0;
    virtual void setVoltage(double leftMillivolts, double rightMillivolts) = 0;
    virtual void stop() = 0;
    virtual ~Drivetrain() = default;
};
```

`tank()` and `arcade()` use normalized commands, usually `[-1.0, 1.0]`.

---

## `TankDrive`

File:

```text
include/studentlib/drivetrain/tank_drive.hpp
src/drivetrain/tank_drive.cpp
```

Concrete differential/tank drive.

Owns:

* left `MotorGroup`
* right `MotorGroup`
* `DrivetrainConfig`

Main API:

```cpp
TankDrive(MotorGroup leftMotors,
          MotorGroup rightMotors,
          DrivetrainConfig config);

void tank(double left, double right) override;
void arcade(double forward, double turn) override;
void setVoltage(double leftMillivolts, double rightMillivolts) override;
void stop() override;

const DrivetrainConfig& getConfig() const;
```

---

# Estimation Layer

## `StateEstimator`

File:

```text
include/studentlib/estimation/state_estimator.hpp
```

Localization abstraction boundary.

```cpp
class StateEstimator {
public:
    virtual Pose2D getPose() const = 0;
    virtual void setPose(const Pose2D& pose) = 0;
    virtual Twist2D getVelocity() const = 0;
    virtual void reset() = 0;
    virtual void step(double dt) = 0;
    virtual ~StateEstimator() = default;
};
```

Motion code depends only on this interface.

---

## `MotionDelta`

File:

```text
include/studentlib/estimation/motion_delta.hpp
src/estimation/motion_delta.cpp
```

Robot-local motion increment:

```cpp
struct MotionDelta {
    double dxLocalInches;
    double dyLocalInches;
    double dthetaRadians;
};
```

This is body-frame motion, not world-frame motion.

---

## `SensorSnapshot`

File:

```text
include/studentlib/estimation/sensor_snapshot.hpp
src/estimation/sensor_snapshot.cpp
```

Coherent sensor readings for estimators.

```cpp
struct IMUSnapshot {
    double headingRadians;
};

struct TrackerWheelSnapshot {
    TrackerAxis axis;
    double distanceInches;
};

struct DistanceSensorSnapshot {
    double distanceInches;
};

struct SensorSnapshot {
    std::optional<IMUSnapshot> imu;
    std::vector<TrackerWheelSnapshot> trackerWheels;
    std::vector<DistanceSensorSnapshot> distanceSensors;
};
```

No raw PROS devices are stored here.

---

## `OdometryEstimator`

File:

```text
include/studentlib/estimation/odometry_estimator.hpp
src/estimation/odometry_estimator.cpp
```

Implements `StateEstimator`.

Responsibilities:

* read `SensorSuite`
* use `RobotModel` metadata
* compute tracker deltas
* use IMU heading delta when available
* integrate pose
* estimate velocity

Supported practical v1 cases:

| Sensors                              | Behavior                         |
| ------------------------------------ | -------------------------------- |
| IMU + longitudinal tracker           | heading + forward motion         |
| IMU + longitudinal + lateral tracker | heading + full local translation |
| two longitudinal trackers + IMU      | averaged forward travel          |
| IMU only                             | heading updates, no translation  |

Odometry integration uses midpoint heading:

```cpp
thetaMid = oldTheta + dtheta / 2

dxWorld = dxLocal * cos(thetaMid) - dyLocal * sin(thetaMid)
dyWorld = dxLocal * sin(thetaMid) + dyLocal * cos(thetaMid)
```

---

# MCL Layer

MCL means Monte Carlo Localization.

It is optional. For most early testing, odometry is simpler.

## `Particle`

File:

```text
include/studentlib/estimation/mcl/particle.hpp
```

```cpp
struct Particle {
    Pose2D pose;
    double weight;
};
```

---

## `MCLConfig`

File:

```text
include/studentlib/estimation/mcl/mcl_config.hpp
src/estimation/mcl/mcl_config.cpp
```

Configuration:

```cpp
struct MCLConfig {
    std::size_t particleCount;
    double translationNoiseStdDevInches;
    double rotationNoiseStdDevRadians;
    double distanceSensorNoiseStdDevInches;
    double headingSensorNoiseStdDevRadians;
    double resampleEffectiveCountThresholdRatio;
    double randomParticleInjectionRatio;
    double extractedPoseSmoothingFactor;

    bool isValid() const;
};
```

---

## `MotionModel`

File:

```text
include/studentlib/estimation/mcl/motion_model.hpp
src/estimation/mcl/motion_model.cpp
```

Applies noisy odometry motion to each particle.

---

## `SensorModel`

File:

```text
include/studentlib/estimation/mcl/sensor_model.hpp
src/estimation/mcl/sensor_model.cpp
```

Scores particles using:

* IMU heading likelihood
* distance sensor likelihood
* field-map raycasts

Distance sensor readings are matched to mounted distance sensors by order.

---

## `Resampler`

File:

```text
include/studentlib/estimation/mcl/resampler.hpp
src/estimation/mcl/resampler.cpp
```

Responsibilities:

* normalize particle weights
* compute effective particle count
* systematic resampling
* optional random particle injection inside field bounds

---

## `MCLEstimator`

File:

```text
include/studentlib/estimation/mcl/mcl_estimator.hpp
src/estimation/mcl/mcl_estimator.cpp
```

Implements `StateEstimator`.

Step flow:

1. capture sensors
2. compute motion delta
3. predict particles
4. score particles
5. normalize weights
6. resample if needed
7. extract pose
8. estimate velocity

Pose extraction uses:

* weighted mean for x/y
* circular mean for heading

---

# Control Layer

## `PIDConfig`

File:

```text
include/studentlib/control/pid_config.hpp
src/control/pid_config.cpp
```

```cpp
struct PIDConfig {
    double kP;
    double kI;
    double kD;
    double integralClamp;
    double outputMin;
    double outputMax;
    double deadband;
    bool antiWindupEnabled;
    bool derivativeOnMeasurement;
    double smallErrorThreshold;
    double smallErrorTimeoutSeconds;

    bool isValid() const;
};
```

---

## `PIDController`

File:

```text
include/studentlib/control/pid_controller.hpp
src/control/pid_controller.cpp
```

Generic scalar PID controller.

Main API:

```cpp
double update(double setpoint, double measurement, double dt);
double update(double setpoint, double measurement, double measurementRate, double dt);

double updateFromError(double error, double dt);
double updateFromError(double error, double errorRate, double dt);
```

The external derivative overloads allow higher layers to supply a known derivative estimate.

Examples:

```cpp
controller.update(setpoint, measurement, measurementRate, dt);
controller.updateFromError(error, errorRate, dt);
```

Derivative behavior:

| Method                                               | Derivative Source          |
| ---------------------------------------------------- | -------------------------- |
| `update(setpoint, measurement, dt)`                  | finite difference          |
| `update(setpoint, measurement, measurementRate, dt)` | supplied measurement rate  |
| `updateFromError(error, dt)`                         | finite difference on error |
| `updateFromError(error, errorRate, dt)`              | supplied error rate        |

---

## `SlewRateLimiter`

File:

```text
include/studentlib/control/slew_rate_limiter.hpp
src/control/slew_rate_limiter.cpp
```

Limits how quickly a scalar command changes.

---

## `LowPassFilter`

File:

```text
include/studentlib/control/low_pass_filter.hpp
src/control/low_pass_filter.cpp
```

First-order scalar low-pass filter:

```cpp
filtered = alpha * input + (1 - alpha) * previous
```

---

## `SettleConfig`

File:

```text
include/studentlib/control/settle_config.hpp
src/control/settle_config.cpp
```

```cpp
struct SettleConfig {
    double positionTolerance;
    double velocityTolerance;
    double settleTimeSeconds;
    double timeoutSeconds;

    bool isValid() const;
};
```

`velocityTolerance < 0` means velocity tolerance is unused.

---

## `MotionConstraints`

File:

```text
include/studentlib/control/motion_constraints.hpp
src/control/motion_constraints.cpp
```

```cpp
struct MotionConstraints {
    double maxSpeed;
    double minSpeed;
    double maxTurnSpeed;
    double maxAcceleration;

    bool isValid() const;
};
```

---

# Motion Layer

## `MotionStatus`

File:

```text
include/studentlib/motion/motion_status.hpp
src/motion/motion_status.cpp
```

```cpp
enum class MotionStatus {
    IDLE,
    RUNNING,
    SETTLED,
    TIMED_OUT,
    CANCELED,
    ERROR
};
```

---

## `MotionCommand`

File:

```text
include/studentlib/motion/motion_command.hpp
src/motion/motion_command.cpp
```

Abstract motion command:

```cpp
class MotionCommand {
public:
    virtual void start() = 0;
    virtual void update(double dt) = 0;
    virtual bool isFinished() const = 0;
    virtual MotionStatus getStatus() const = 0;
    virtual void cancel() = 0;
    virtual ~MotionCommand() = default;
};
```

---

## `motion_params.hpp`

Defines parameter structs:

```cpp
TurnToHeadingParams
TurnToPointParams
DriveToPointParams
MoveToPoseParams
QuadraticBezierPathParams
MoveToPosePathMode
```

### `TurnToHeadingParams`

```cpp
PIDConfig angular;
double maxSpeed;
double minSpeed;
SettleConfig settle;
```

### `DriveToPointParams`

```cpp
PIDConfig linear;
PIDConfig angular;
double maxSpeed;
double minSpeed;
bool reverse;
SettleConfig settle;
```

### `MoveToPoseParams`

```cpp
PIDConfig linear;
PIDConfig angular;
double maxSpeed;
double minSpeed;
bool reverse;
double approachHeadingWeight;
double maxFinalAngleError;
SettleConfig settle;

MoveToPosePathMode pathMode;
QuadraticBezierPathParams quadraticBezier;
```

---

# Motion Commands

## `TurnToHeading`

Turns robot to an absolute heading.

```cpp
robot.turnToHeading(1.57, params);
```

Uses:

```cpp
heading_error = normalizeAngle(targetHeading - currentHeading)
```

Commands:

```cpp
tank(-turn, turn)
```

Settles when heading error stays within tolerance for the configured settle time.

---

## `TurnToPoint`

Turns robot to face a point.

```cpp
robot.turnToPoint(Point2D{48.0, 24.0}, params);
```

Computes:

```cpp
desired_heading = atan2(target.y - pose.y, target.x - pose.x)
heading_error = normalizeAngle(desired_heading - pose.theta)
```

---

## `DriveToPoint`

Drives to a target point.

```cpp
robot.driveToPoint(Point2D{24.0, 0.0}, params);
```

Uses:

* linear PID on distance error
* angular PID on heading error

Reverse mode:

```cpp
params.reverse = true;
```

In reverse mode, the desired heading is shifted by `pi`, and the linear command is negated.

---

## `MoveToPose`

High-level point-to-pose command.

```cpp
robot.moveToPose(Pose2D{24.0, 12.0, 1.57}, params);
```

Settles when both are true:

* distance to final target position is within tolerance
* final heading error is within `maxFinalAngleError`

`MoveToPose` supports multiple path/reference modes.

---

# MoveToPose Path Modes

## `POSE_REGULATOR`

```cpp
params.pathMode = MoveToPosePathMode::POSE_REGULATOR;
```

Behavior:

* regulates distance to target point
* blends between approach heading and final heading
* useful as a general direct point-to-pose controller

Best for:

* simple autonomous motions
* direct corrections
* early tuning

---

## `STRAIGHT_THEN_TURN`

```cpp
params.pathMode = MoveToPosePathMode::STRAIGHT_THEN_TURN;
```

Behavior:

1. drive to the target position
2. turn to the final heading

Best for:

* predictable non-holonomic behavior
* simple tuning
* students first learning autonomous motion
* cases where curved approach is unnecessary

---

## `QUADRATIC_BEZIER`

```cpp
params.pathMode = MoveToPosePathMode::QUADRATIC_BEZIER;
```

Behavior:

* creates a quadratic Bézier reference path
* tracks a lookahead point on the path
* uses tangent heading until close to the final pose
* then switches toward final target heading

Best for:

* smoother approach paths
* chaining motions
* approaching a target pose from a desired direction

---

# Quadratic Bézier Path

File:

```text
include/studentlib/motion/quadratic_bezier_path.hpp
src/motion/quadratic_bezier_path.cpp
```

A geometric path/reference helper, not a controller.

Quadratic curve:

```text
B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2
```

Where:

| Point | Meaning         |
| ----- | --------------- |
| `P0`  | start position  |
| `P1`  | control point   |
| `P2`  | target position |

The control point is placed behind the target pose along the target heading.

Main API:

```cpp
Point2D samplePoint(double t) const;
Vector2D sampleTangent(double t) const;
double sampleHeading(double t) const;

double findNearestT(
    const Point2D& position,
    double tMin,
    double tMax,
    std::size_t samples = 25) const;
```

Configuration:

```cpp
struct QuadraticBezierPathParams {
    double leadFactor;
    double searchWindowT;
    double lookaheadT;
    double finalHeadingSwitchDistance;
};
```

---

# MotionManager

File:

```text
include/studentlib/motion/motion_manager.hpp
src/motion/motion_manager.cpp
```

Owns one active motion command at a time.

Main API:

```cpp
void update(double dt);
void cancel();

MotionStatus getStatus() const;
bool isBusy() const;
bool isFinished() const;

void startTurnToHeading(...);
void startTurnToPoint(...);
void startDriveToPoint(...);
void startMoveToPose(...);
```

`MotionManager` does not create tasks. It is updated by `Robot`'s persistent motion task.

---

# Robot Layer

## `RobotConfig`

File:

```text
include/studentlib/robot/robot_config.hpp
src/robot/robot_config.cpp
```

```cpp
enum class LocalizationMode {
    ODOMETRY,
    MCL
};

enum class ChassisModel {
    DIFFERENTIAL,
    HOLONOMIC
};

struct RobotConfig {
    LocalizationMode localizationMode;
    ChassisModel chassisModel;
    double localizationPeriodSeconds;
    double motionPeriodSeconds;

    bool isValid() const;
};
```

`HOLONOMIC` is only a future hook in v1. The current implementation is differential/tank-drive focused.

---

## `Robot`

File:

```text
include/studentlib/robot/robot.hpp
src/robot/robot.cpp
```

The main user-facing facade.

Owns:

* `SensorSuite`
* `RobotModel`
* optional `FieldMap`
* `std::unique_ptr<Drivetrain>`
* `std::unique_ptr<StateEstimator>`
* `MotionManager`
* persistent localization task
* persistent motion task

Main lifecycle API:

```cpp
void initialize();
void start();
void stop();
```

Pose API:

```cpp
Pose2D getPose() const;
void setPose(const Pose2D& pose);
void resetPose();
```

Blocking motion API:

```cpp
void moveToPose(const Pose2D& target, const MoveToPoseParams& params = {});
void turnToPoint(const Point2D& target, const TurnToPointParams& params = {});
void turnToHeading(double headingRadians, const TurnToHeadingParams& params = {});
void driveToPoint(const Point2D& target, const DriveToPointParams& params = {});
```

Nonblocking motion API:

```cpp
void startMoveToPose(const Pose2D& target, const MoveToPoseParams& params = {});
void startTurnToPoint(const Point2D& target, const TurnToPointParams& params = {});
void startTurnToHeading(double headingRadians, const TurnToHeadingParams& params = {});
void startDriveToPoint(const Point2D& target, const DriveToPointParams& params = {});
```

Motion status API:

```cpp
MotionStatus getMotionStatus() const;
bool isMotionComplete() const;
bool isMotionBusy() const;
void cancelMotion();
void waitUntilSettled();
```

Manual drive passthrough:

```cpp
void tank(double left, double right);
void arcade(double forward, double turn);
```

---

# API / Builder Layer

## `Builder`

File:

```text
include/studentlib/api/builder.hpp
src/api/builder.cpp
```

Readable robot construction helper.

Example:

```cpp
auto robot = Builder()
    .withTankDrive({1, 2, 3}, {-4, -5, -6}, 11.5, 1.625)
    .withIMU(7)
    .withVerticalTracker(8, 1.0, 1.0, Point2D{0.0, 0.0}, false)
    .withHorizontalTracker(9, 1.0, 1.0, Point2D{0.0, 0.0}, false)
    .withLocalizationMode(LocalizationMode::ODOMETRY)
    .build();
```

Important current API note:

* `Builder::build()` returns a `Robot` by value.
* This is fine for direct local construction with guaranteed copy elision.
* Do not write helper functions that create a named local `Robot` and then `return robot;`.
* For persistent program-lifetime robots, direct `Robot` construction inside `std::make_unique<Robot>(...)` is often cleaner.

---

# Configuration Patterns

## Minimal Tank Robot

Use when:

* testing motor directions
* testing manual drive
* first bring-up

Typical sensors:

* drive motors only
* maybe IMU

Limitations:

* no useful translation odometry unless trackers are installed
* autonomous pose motion will be limited

---

## Odometry Robot

Use when:

* you want reliable autonomous pose tracking
* you have tracking wheels and IMU

Recommended sensors:

* IMU
* one longitudinal tracker
* one lateral tracker

Example configuration:

```cpp
.withIMU(7)
.withVerticalTracker(8, 1.0, 1.0, Point2D{0.0, 0.0}, false)
.withHorizontalTracker(9, 1.0, 1.0, Point2D{0.0, 0.0}, false)
.withLocalizationMode(LocalizationMode::ODOMETRY)
```

---

## MCL Robot

Use when:

* odometry drift needs correction
* distance sensors can see known field walls/objects
* a field map is available

Required additions:

* distance sensors
* `FieldMap`
* `LocalizationMode::MCL`

If MCL is requested without a field map, the current `Robot` implementation falls back to odometry.

---

## Hypothetical Differential Robot Configuration

Example robot:

```text
Left motors:  1, 2, 3
Right motors: 4, 5, 6 reversed
IMU:          7
Vertical tracker:   8
Horizontal tracker: 9
Track width: 11.5 in
Drive wheel radius: 1.625 in
Tracker wheel radius: 1.0 in
```

Because the current motor API supports negative ports as reversed motors:

```cpp
std::vector<int> left_motor_ports{1, 2, 3};
std::vector<int> right_motor_ports{-4, -5, -6};
```

---

# Recommended `main.cpp` Structure

Keep `main.cpp` simple.

Recommended files:

```text
src/
├── main.cpp
├── support.hpp
└── support.cpp
```

## `support.cpp`

Put these in `support.cpp`:

* motor ports
* reversed motor convention
* track width
* wheel radius
* tracker geometry
* IMU config
* robot construction
* LCD helpers
* joystick scaling helpers

## `main.cpp`

Keep only:

* PROS hooks
* autonomous command sequence
* opcontrol loop
* PID/motion parameter helpers

Example:

```cpp
void autonomous() {
    Robot& robot = support::robot();

    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    robot.turnToHeading(0.5, turnParams());
    robot.driveToPoint(Point2D{8.0, 0.0}, driveParams());

    robot.moveToPose(
        Pose2D{12.0, 6.0, 0.0},
        moveParams(MoveToPosePathMode::STRAIGHT_THEN_TURN));

    robot.moveToPose(
        Pose2D{18.0, 10.0, 1.5708},
        moveParams(MoveToPosePathMode::QUADRATIC_BEZIER));
}
```

---

# Conservative Starting Parameters

## TurnToHeading

```cpp
TurnToHeadingParams params;

params.angular.kP = 0.6;
params.angular.kD = 0.02;

params.maxSpeed = 0.30;
params.minSpeed = 0.06;

params.settle.positionTolerance = 0.08;
params.settle.settleTimeSeconds = 0.20;
params.settle.timeoutSeconds = 2.0;
```

## DriveToPoint

```cpp
DriveToPointParams params;

params.linear.kP = 0.06;
params.linear.kD = 0.002;

params.angular.kP = 0.6;
params.angular.kD = 0.02;

params.maxSpeed = 0.35;
params.minSpeed = 0.08;

params.settle.positionTolerance = 1.0;
params.settle.settleTimeSeconds = 0.20;
params.settle.timeoutSeconds = 2.5;
```

## MoveToPose

```cpp
MoveToPoseParams params;

params.linear.kP = 0.06;
params.linear.kD = 0.002;

params.angular.kP = 0.6;
params.angular.kD = 0.02;

params.maxSpeed = 0.35;
params.minSpeed = 0.08;

params.maxFinalAngleError = 0.12;

params.settle.positionTolerance = 1.0;
params.settle.settleTimeSeconds = 0.20;
params.settle.timeoutSeconds = 2.5;

params.quadraticBezier.leadFactor = 0.40;
params.quadraticBezier.searchWindowT = 0.20;
params.quadraticBezier.lookaheadT = 0.05;
params.quadraticBezier.finalHeadingSwitchDistance = 6.0;
```

---

# Bring-Up Order

## 1. Motor Direction Test

Put robot on blocks.

Command:

```cpp
robot.tank(0.2, 0.2);
```

Expected:

* both sides spin forward

If not:

* reverse the incorrect motor ports
* check left/right motor group assignment

---

## 2. IMU Heading Test

Reset pose:

```cpp
robot.setPose(Pose2D{0.0, 0.0, 0.0});
```

Rotate robot counterclockwise by hand.

Expected:

* theta increases

If not:

* check IMU orientation
* check yaw offset/sign convention

---

## 3. Tracker Sign Test

Push robot forward by hand.

Expected:

* x increases when heading is near zero

Push robot left by hand.

Expected:

* y increases if lateral tracker is configured positive-left

If not:

* reverse tracker sensor
* check `TrackerAxis`
* check tracker mounting assumptions

---

## 4. Localization-Only Test

Before autonomous motion, print pose continuously.

Check:

* pose reset works
* heading sign works
* forward movement changes x correctly
* lateral movement changes y correctly

---

## 5. Motion Smoke Test

Start small:

```cpp
robot.turnToHeading(0.5, turnParams());
robot.driveToPoint(Point2D{8.0, 0.0}, driveParams());
```

Then test:

```cpp
robot.moveToPose(Pose2D{12.0, 6.0, 0.0}, moveParams(...));
```

---

# Troubleshooting

## Robot turns the wrong direction

Likely causes:

* left/right motor groups swapped
* right side not reversed correctly
* IMU sign convention mismatch
* angular PID sign issue

## Forward command drives backward

Likely causes:

* both motor groups reversed
* pose heading initialized incorrectly
* vertical tracker sign reversed

## Pose x decreases when pushing forward

Likely causes:

* vertical tracker reversed
* tracker gear ratio sign wrong
* heading estimate is 180 degrees off

## x and y appear swapped

Likely causes:

* vertical and horizontal trackers swapped
* wrong `TrackerAxis`
* robot-frame convention mismatch

## Robot overshoots

Likely causes:

* kP too high
* kD too low
* maxSpeed too high
* localization lag/noise

## Robot never settles

Likely causes:

* position tolerance too tight
* angle tolerance too tight
* velocity tolerance too strict
* localization noise near target
* minSpeed too high causing oscillation

## Robot times out unexpectedly

Likely causes:

* timeout too short
* maxSpeed too low
* minSpeed too low to overcome friction
* target too far
* localization sign convention wrong

## Bézier path curves too aggressively

Try:

```cpp
params.quadraticBezier.leadFactor = 0.25;
```

## Bézier path feels sluggish

Try:

```cpp
params.quadraticBezier.lookaheadT = 0.08;
```

Increase carefully.

---

# Current Limitations

StudentLib v1 intentionally does not include:

* spline generation beyond quadratic Bézier reference support
* pure pursuit
* Ramsete
* SLAM
* map building
* holonomic drivetrain implementation
* mechanism subsystem framework
* advanced action scheduler
* parallel command graph

The focus is a clear, working, teachable differential-drive library.

---

# Recommended First Autonomous Test

```cpp
void autonomous() {
    Robot& robot = support::robot();

    robot.setPose(Pose2D{0.0, 0.0, 0.0});

    robot.turnToHeading(0.5, turnParams());

    robot.driveToPoint(
        Point2D{8.0, 0.0},
        driveParams());

    robot.moveToPose(
        Pose2D{12.0, 6.0, 0.0},
        moveParams(MoveToPosePathMode::STRAIGHT_THEN_TURN));

    robot.moveToPose(
        Pose2D{18.0, 10.0, 1.5708},
        moveParams(MoveToPosePathMode::QUADRATIC_BEZIER));
}
```

Start with low speed and short distances. Validate localization before trusting motion.

```
```
