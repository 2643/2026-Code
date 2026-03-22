# Getting Limelight Pose vs Robot Telemetry Pose

## Overview

You have **two different pose sources**:

1. **Limelight Pose** - Raw vision measurement from April Tags
2. **Robot Telemetry Pose** - Fused odometry (wheels + gyro + vision corrections)

Understanding both is critical for diagnosing your pose-based turret aiming!

---

## Quick Answer: How to Get Both Poses

### Limelight Pose (Raw Vision)

```java
// Get Limelight's raw pose estimate using MegaTag2
var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");

// Extract the Pose2d
Pose2d limelightPose = llMeasurement.pose;

// Get individual components
double limelightX = limelightPose.getX();
double limelightY = limelightPose.getY();
double limelightRotation = limelightPose.getRotation().getDegrees();
```

### Robot Telemetry Pose (Fused Odometry)

```java
// Get robot's fused pose from swerve odometry
var driveState = RobotContainer.drivetrain.getState();
Pose2d robotPose = driveState.Pose;

// Get individual components
double robotX = robotPose.getX();
double robotY = robotPose.getY();
double robotRotation = robotPose.getRotation().getDegrees();
```

---

## Where These Come From

### Limelight Pose (Raw Vision)

```
April Tags on field
        ↓
Limelight detects tags
        ↓
LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2()
        ↓
Returns PoseEstimate with:
├─ pose: Pose2d (x, y, rotation)
├─ tagCount: How many tags were seen
├─ timestamp: When measurement was taken
└─ latency: Network latency
```

**What it is:** Pure vision-only estimate, no wheel data mixed in

### Robot Telemetry Pose (Fused)

```
Wheel encoders (measure distance traveled)
Gyroscope (measure rotation)
        ↓
SwerveDriveOdometry calculates position
        ↓
Vision.updateRobotPoseFromLimelight() corrects drift
        ↓
RobotContainer.drivetrain.addVisionMeasurement()
        ↓
Kalman Filter fuses measurements
        ↓
getState().Pose returns best estimate
```

**What it is:** Hybrid estimate combining wheels + gyro + vision

---

## Why You Need Both

### Scenario: Diagnosing Turret Aim Problems

```
Robot at (3.0, 2.0) according to odometry
Robot at (3.2, 2.1) according to Limelight
Turret aims at wrong spot

Questions:
1. Is the difference > 0.2m? → Odometry drifted significantly
2. Is Limelight seeing April Tags? → Check tag count
3. Which pose is actually correct? → Compare to where robot PHYSICALLY is
4. Should we trust vision or wheels more right now? → Depends on tag visibility
```

---

## Code Examples: Using Both Poses

### Example 1: Compare Odometry vs Vision

```java
// In Vision.java or any subsystem

public void compareOdometryAndVision() {
  // Get both poses
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d odometryPose = driveState.Pose;
  
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");
  Pose2d visionPose = llMeasurement.pose;
  
  // Calculate difference
  double poseDifferenceX = odometryPose.getX() - visionPose.getX();
  double poseDifferenceY = odometryPose.getY() - visionPose.getY();
  double poseDifferenceDistance = Math.hypot(poseDifferenceX, poseDifferenceY);
  
  // Log to dashboard
  SmartDashboard.putNumber("Odometry X", odometryPose.getX());
  SmartDashboard.putNumber("Odometry Y", odometryPose.getY());
  SmartDashboard.putNumber("Vision X", visionPose.getX());
  SmartDashboard.putNumber("Vision Y", visionPose.getY());
  SmartDashboard.putNumber("Pose Difference (meters)", poseDifferenceDistance);
  SmartDashboard.putString("Vision Status", 
    llMeasurement.tagCount > 0 ? "Detecting tags" : "No tags");
}
```

**What to expect:**
- Without vision: difference grows over time (odometry drifts)
- With vision: difference stays small (< 0.2m)
- If difference is huge: starting pose was very wrong

### Example 2: Use Best Estimate for Turret Aiming

```java
// In Swivel.java or turret command

public void aimUsingBestPose() {
  // Get both poses
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d robotPose = driveState.Pose;  // This is the FUSED pose (most reliable)
  
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");
  Pose2d limelightPose = llMeasurement.pose;
  
  // Use fused pose for actual aiming (already includes vision corrections)
  // This is what you're already doing in turretTrackPose()
  turretTrackPose(robotPose);  // ← Use the FUSED pose
  
  // But log the raw vision too for diagnostics
  SmartDashboard.putNumber("Raw Vision X", limelightPose.getX());
  SmartDashboard.putNumber("Raw Vision Y", limelightPose.getY());
  SmartDashboard.putNumber("Fused Odometry X", robotPose.getX());
  SmartDashboard.putNumber("Fused Odometry Y", robotPose.getY());
}
```

### Example 3: Weighted Pose Selection

```java
// Use vision if available, fall back to odometry if not

public Pose2d getBestRobotPose() {
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d odometryPose = driveState.Pose;
  
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");
  
  // If vision is available and confident, trust it more
  if (llMeasurement != null && llMeasurement.tagCount >= 2) {
    // 2+ tags = high confidence
    return llMeasurement.pose;
  } else if (llMeasurement != null && llMeasurement.tagCount == 1) {
    // 1 tag = medium confidence, average with odometry
    double avgX = (odometryPose.getX() + llMeasurement.pose.getX()) / 2.0;
    double avgY = (odometryPose.getY() + llMeasurement.pose.getY()) / 2.0;
    return new Pose2d(avgX, avgY, odometryPose.getRotation());
  } else {
    // No vision, use odometry only
    return odometryPose;
  }
}
```

---

## Understanding Your Current Code

### What Vision.java Does (Already Implemented!)

```java
// In Vision.updateRobotPoseFromLimelight():

var driveState = RobotContainer.drivetrain.getState();
double headingDeg = driveState.Pose.getRotation().getDegrees();  // ← Odometry pose

// Tell Limelight which way robot is facing
LimelightHelpers.SetRobotOrientation(limelightName1, headingDeg, 0, 0, 0, 0, 0);

// Get Limelight's measurement
var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);

// Fuse it with odometry
if (llMeasurement != null && llMeasurement.tagCount > 0) {
  RobotContainer.drivetrain.addVisionMeasurement(
      llMeasurement.pose,        // ← Raw Limelight pose
      llMeasurement.timestampSeconds
  );
  // After this call, getState().Pose includes the vision correction!
}
```

**Flow:**
1. Get odometry pose (wheels + gyro only)
2. Get Limelight raw pose (vision only)
3. Fuse them together
4. Return best estimate via `getState().Pose`

---

## Telemetry You Should Log

Add this to your periodic() to monitor both poses:

```java
@Override
public void periodic() {
  updateData();
  updateRobotPoseFromLimelight();
  
  // === ODOMETRY (Wheel-based) ===
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d odoPose = driveState.Pose;
  SmartDashboard.putNumber("Odometry/X", odoPose.getX());
  SmartDashboard.putNumber("Odometry/Y", odoPose.getY());
  SmartDashboard.putNumber("Odometry/Rotation", odoPose.getRotation().getDegrees());
  
  // === VISION (Limelight only) ===
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");
  if (llMeasurement != null) {
    SmartDashboard.putNumber("Vision/X", llMeasurement.pose.getX());
    SmartDashboard.putNumber("Vision/Y", llMeasurement.pose.getY());
    SmartDashboard.putNumber("Vision/Rotation", llMeasurement.pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/Tag Count", llMeasurement.tagCount);
  }
  
  // === DIFFERENCE ===
  if (llMeasurement != null && llMeasurement.tagCount > 0) {
    double diffX = odoPose.getX() - llMeasurement.pose.getX();
    double diffY = odoPose.getY() - llMeasurement.pose.getY();
    double diffDist = Math.hypot(diffX, diffY);
    SmartDashboard.putNumber("Pose Difference/X", diffX);
    SmartDashboard.putNumber("Pose Difference/Y", diffY);
    SmartDashboard.putNumber("Pose Difference/Distance (m)", diffDist);
  }
}
```

---

## On SmartDashboard, You'll See:

### Good Telemetry (Fusion Working)

```
Odometry/X:              3.544 m
Odometry/Y:              4.025 m
Odometry/Rotation:       180.0 °

Vision/X:                3.542 m
Vision/Y:                4.027 m
Vision/Rotation:         179.8 °
Vision/Tag Count:        2 (Two April Tags seen)

Pose Difference/Distance: 0.003 m (< 0.2m = GOOD!)
```

**Interpretation:** Vision and odometry agree. Fusion is working!

### Bad Telemetry (Odometry Drifting)

```
Odometry/X:              5.200 m
Odometry/Y:              3.800 m
Odometry/Rotation:       185.0 °

Vision/X:                4.923 m
Vision/Y:                4.156 m
Vision/Rotation:         179.5 °
Vision/Tag Count:        2 (Tags are visible)

Pose Difference/Distance: 0.380 m (> 0.2m = BAD!)
```

**Interpretation:** Odometry has drifted. Vision should correct it on next measurement.

### No Vision Telemetry

```
Odometry/X:              3.544 m
Odometry/Y:              4.025 m

Vision/X:                -- (no measurement)
Vision/Y:                -- (no measurement)
Vision/Tag Count:        0 (No tags detected)
```

**Interpretation:** Limelight can't see April Tags. Odometry will drift over time.

---

## Which Pose Should You Use?

### For Turret Aiming → Use `getState().Pose` (Fused)

```java
// ✅ CORRECT - Use the fused pose
Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;
turretTrackPose(robotPose);  // This includes vision corrections
```

**Why:** The fused pose already includes vision corrections via the Kalman filter. It's the "best estimate" of where the robot really is.

### For Diagnostics → Log Both

```java
// ✅ CORRECT - Log both for comparison
Pose2d odometry = RobotContainer.drivetrain.getState().Pose;
Pose2d rawVision = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen").pose;

SmartDashboard.putNumber("Fused Pose X", odometry.getX());
SmartDashboard.putNumber("Raw Vision X", rawVision.getX());
```

**Why:** Comparing them tells you if vision is helping (difference shrinks) or if starting pose was wrong (difference is huge).

---

## Practical Example: Finding Position Using Both Poses

### Scenario: Robot Gets Lost (Lost Track of Position)

```java
public void reestablishPosition() {
  // 1. Get current odometry (might be wrong due to drift)
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d odometryPose = driveState.Pose;
  System.out.println("Odometry says: (" + odometryPose.getX() + ", " + odometryPose.getY() + ")");
  
  // 2. Get Limelight raw pose (only works if tags visible)
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-allen");
  
  if (llMeasurement != null && llMeasurement.tagCount >= 2) {
    // 3a. If vision is confident, use it
    Pose2d bestPose = llMeasurement.pose;
    System.out.println("Vision says: (" + bestPose.getX() + ", " + bestPose.getY() + ")");
    System.out.println("Trust VISION - has " + llMeasurement.tagCount + " tags");
    
    // Optional: Reset odometry to vision pose to prevent future drift
    RobotContainer.drivetrain.resetPose(bestPose);
  } else {
    // 3b. If vision not available, use odometry but acknowledge it drifted
    System.out.println("No vision available - odometry may have drifted");
    System.out.println("Using odometry: (" + odometryPose.getX() + ", " + odometryPose.getY() + ")");
  }
}
```

---

## Summary Table

| Aspect | Limelight Pose | Robot Telemetry Pose |
|--------|----------------|----------------------|
| **From** | April Tags on field | Wheels + gyro + vision |
| **Get it with** | `LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2()` | `RobotContainer.drivetrain.getState().Pose` |
| **Accuracy** | ±0.1m (with tags) | ±0.2m (growing over time) |
| **Updates** | ~10 Hz (when tags visible) | 200 Hz (always) |
| **For Aiming** | Use for diagnostics | ✅ Use this |
| **For Debugging** | Log to SmartDashboard | Log to SmartDashboard |
| **Use When** | Verifying vision fusion | Actually controlling robot |

---

## Next Steps

1. **Add telemetry** to log both poses to SmartDashboard
2. **Test in gym** and watch the values
3. **Compare** - they should be close when vision is working
4. **Use fused pose** for turret aiming (you're already doing this!)
5. **Use raw vision** for diagnostics only

This will give you confidence that your pose fusion is working correctly!
