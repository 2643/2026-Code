# Where to Put the Pose Comparison Code: Swivel or Hood?

## Answer: **SWIVEL** (with some notes about Hood)

Here's why:

---

## Responsibility Breakdown

### Swivel (Turret Rotation)
**Purpose:** Aim the turret left/right at the target

```java
// Swivel is responsible for:
✓ Calculating angle to target
✓ Using robot pose to determine aim direction
✓ Already has turretTrackPose() which uses robot pose
✓ Already has periodic() logging turret telemetry
✓ Needs to compare robot pose vs vision pose for debugging
```

### Hood (Elevation Angle)
**Purpose:** Adjust shooting angle up/down based on distance

```java
// Hood is responsible for:
✓ Calculating hood angle based on distance
✓ Using Limelight target area to estimate distance
✓ autoPitch() method that uses vision data
✗ Doesn't need to compare full robot poses (it uses distance only)
```

---

## Why SWIVEL Is the Right Place

### Current Swivel.java Structure

Your `Swivel.java` already has:

```java
public void turretTrackPose(Pose2d target) {
  // Gets robot pose
  Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;
  
  // Gets Limelight pose
  m_limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
  m_fusedRobotPose = robotPose;
  
  // Calculates angle to target
  // ...
}

@Override
public void periodic() {
  // Logs all telemetry including:
  // - Robot X/Y (Fused)
  // - Limelight X/Y (Raw)
  // - Turret angles
  // - Distance to hub
}
```

**This is EXACTLY where pose comparison belongs!**

---

## Where to Add the Code

### Option 1: Add to Swivel.periodic() (BEST)

```java
@Override
public void periodic() {
  // Existing telemetry...
  
  // === NEW: POSE COMPARISON FOR DEBUGGING ===
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d robotPose = driveState.Pose;
  
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);
  
  if (llMeasurement != null && llMeasurement.tagCount > 0) {
    // Calculate difference
    double diffX = Math.abs(robotPose.getX() - llMeasurement.pose.getX());
    double diffY = Math.abs(robotPose.getY() - llMeasurement.pose.getY());
    double diffDistance = Math.hypot(diffX, diffY);
    
    // Log to dashboard
    SmartDashboard.putNumber("Pose Difference X (m)", diffX);
    SmartDashboard.putNumber("Pose Difference Y (m)", diffY);
    SmartDashboard.putNumber("Pose Difference Total (m)", diffDistance);
    
    // Verify fusion quality
    if (diffDistance < 0.1) {
      SmartDashboard.putString("Fusion Status", "EXCELLENT (both agree)");
    } else if (diffDistance < 0.3) {
      SmartDashboard.putString("Fusion Status", "GOOD (minor drift)");
    } else if (diffDistance < 0.5) {
      SmartDashboard.putString("Fusion Status", "WARNING (odometry drifting)");
    } else {
      SmartDashboard.putString("Fusion Status", "ERROR (starting pose wrong?)");
    }
  } else {
    SmartDashboard.putString("Fusion Status", "No vision - using odometry only");
  }
}
```

### Option 2: Create Helper Method in Swivel

```java
public Pose2d getActualRobotPosition() {
  // Return the best guess of actual position using both measurements
  
  var driveState = RobotContainer.drivetrain.getState();
  Pose2d robotPose = driveState.Pose;
  
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);
  
  if (llMeasurement != null && llMeasurement.tagCount >= 2) {
    // High confidence: 70% vision, 30% odometry
    double actualX = (llMeasurement.pose.getX() * 0.7) + (robotPose.getX() * 0.3);
    double actualY = (llMeasurement.pose.getY() * 0.7) + (robotPose.getY() * 0.3);
    return new Pose2d(actualX, actualY, robotPose.getRotation());
    
  } else if (llMeasurement != null && llMeasurement.tagCount == 1) {
    // Medium confidence: 50/50
    double actualX = (llMeasurement.pose.getX() * 0.5) + (robotPose.getX() * 0.5);
    double actualY = (llMeasurement.pose.getY() * 0.5) + (robotPose.getY() * 0.5);
    return new Pose2d(actualX, actualY, robotPose.getRotation());
    
  } else {
    // No vision: use odometry only
    return robotPose;
  }
}

// Then in periodic():
Pose2d actualPosition = getActualRobotPosition();
SmartDashboard.putNumber("Actual Position X", actualPosition.getX());
SmartDashboard.putNumber("Actual Position Y", actualPosition.getY());
```

---

## NOT Hood

### Why NOT Hood:

```java
// Hood doesn't need pose comparison because:

✗ Hood uses: Limelight target AREA to estimate distance
  └─ Not the full pose, just one number (area)

✗ Hood doesn't aim at a known location
  └─ It's based on "how far away is this target based on its size?"

✗ Hood's autoPitch() just does:
  roundedArea = Math.log(1/area);
  angle = (1.38693*roundedArea)-1.13255;
  └─ Simple calculation, doesn't need pose comparison
```

If you put pose comparison in Hood, it would be out of place since Hood doesn't use the full robot pose in its calculations.

---

## Summary: Where to Put Code

| Code Type | Where | Why |
|-----------|-------|-----|
| Pose comparison | **Swivel.periodic()** | Swivel uses robot pose for aiming |
| Get actual position | **Swivel method** | Swivel needs it for turret angles |
| Distance-based hood | **Hood.autoPitch()** | Hood uses distance, not position |
| Log telemetry | **Both** | Both should log their data |

---

## Complete Example: Where to Add in Swivel

```java
public class Swivel extends SubsystemBase {
  // ... existing fields ...
  
  @Override
  public void periodic() {
    // Existing code...
    
    // === EXISTING TELEMETRY ===
    // Robot odometry, Limelight, turret angles...
    
    // === NEW: POSE COMPARISON (Debug fusion quality) ===
    var driveState = RobotContainer.drivetrain.getState();
    Pose2d robotPose = driveState.Pose;
    
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);
    
    if (llMeasurement != null && llMeasurement.tagCount > 0) {
      double diffX = Math.abs(robotPose.getX() - llMeasurement.pose.getX());
      double diffY = Math.abs(robotPose.getY() - llMeasurement.pose.getY());
      double totalDiff = Math.hypot(diffX, diffY);
      
      SmartDashboard.putNumber("Pose Diff X", diffX);
      SmartDashboard.putNumber("Pose Diff Y", diffY);
      SmartDashboard.putNumber("Pose Diff Total", totalDiff);
      
      // Diagnose fusion health
      if (totalDiff < 0.1) {
        SmartDashboard.putString("Fusion Health", "EXCELLENT");
      } else if (totalDiff > 0.5) {
        SmartDashboard.putString("Fusion Health", "ERROR - Starting pose wrong?");
      }
    }
  }
  
  // Optional: Helper method
  public Pose2d getActualPosition() {
    var driveState = RobotContainer.drivetrain.getState();
    Pose2d robotPose = driveState.Pose;
    
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);
    
    if (llMeasurement != null && llMeasurement.tagCount >= 2) {
      double x = (llMeasurement.pose.getX() * 0.7) + (robotPose.getX() * 0.3);
      double y = (llMeasurement.pose.getY() * 0.7) + (robotPose.getY() * 0.3);
      return new Pose2d(x, y, robotPose.getRotation());
    }
    return robotPose;
  }
}
```

---

## Bottom Line

- ✅ **Swivel:** Use it for pose comparison and finding actual position
- ✅ **Hood:** Keep it focused on distance-based hood angle adjustment
- ✅ **Vision:** Keep providing the raw measurements (already done)
- ✅ **Swerve:** Keep doing the Kalman filter fusion (already done)

Each subsystem stays focused on its job!
