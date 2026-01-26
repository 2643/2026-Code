# How to Use the Shooter Angle System on Your Robot

## 🎯 Overview

Your shooter angle system uses physics to automatically calculate the hood angle needed to score from any distance. The Limelight detects the AprilTag, and the system adjusts the hood angle automatically.

---

## 📋 Step-by-Step Setup Guide

### STEP 1: Measure Your Robot's Physical Parameters

You need to measure these three critical values:

#### A) Ball Velocity (`BALL_VELOCITY_MPS`)

**Method 1 - Radar Gun (Best):**
- Use a sports radar gun
- Shoot balls at a fixed power/RPM
- Record the velocity in mph
- Convert to m/s: `velocity_mps = velocity_mph / 2.237`

**Method 2 - Distance/Time:**
1. Shoot ball horizontally across a known distance (like 10 meters)
2. Time how long it takes
3. Calculate: `velocity = distance / time`

**Example:** Ball travels 10m in 0.8 seconds → `10 / 0.8 = 12.5 m/s`

**Typical FRC values:** 10-18 m/s (22-40 mph)

#### B) Height Delta (`TARGET_HEIGHT_DELTA_M`)

1. Measure height of target opening (center) from ground
2. Measure height of your shooter exit from ground
3. Calculate: `delta = target_height - shooter_height`

**Example:**
- Target center: 2.1m from ground
- Shooter exit: 0.5m from ground  
- Delta: `2.1 - 0.5 = 1.6m`

**Tip:** Keep units in meters!

#### C) Impact Angle (`IMPACT_ANGLE_RAD`)

This is how steep you want the ball to enter the target:

- **-30° to -45°**: Moderate arc, good for most situations
- **-45° to -60°**: Steeper arc, ball drops in more
- **-60° to -75°**: Very steep, almost dropping straight down

**Start with -45°** and adjust based on testing.

---

### STEP 2: Update Constants.java

Open `Constants.java` and update these three values:

```java
public static final double BALL_VELOCITY_MPS = 12.5; // Your measured value
public static final double TARGET_HEIGHT_DELTA_M = 1.6; // Your measured value  
public static final double IMPACT_ANGLE_RAD = Math.toRadians(-45); // Your chosen angle
```

---

### STEP 3: Calibrate Hood Gearing

#### Find Gear Ratio:

**Option A - Physical Measurement:**
1. Reset hood encoder to 0
2. Manually move hood from 0° to exactly 45° (use a protractor/angle finder)
3. Read encoder value (e.g., 25.3 rotations)
4. Calculate: `MOTOR_ROTATIONS_PER_DEGREE = encoder_value / 45.0`
   - Example: `25.3 / 45.0 = 0.562`

**Option B - Gear Calculation:**
```
If your mechanism uses gears/chain:
MOTOR_ROTATIONS_PER_DEGREE = (motor_gear_teeth / hood_gear_teeth) / 360.0
```

#### Update Constants:

```java
public static final double MOTOR_ROTATIONS_PER_DEGREE = 0.562; // Your value
public static final double MIN_HOOD_ANGLE_DEG = 49; // Already set
public static final double MAX_HOOD_ANGLE_DEG = 74; // Already set
```

---

### STEP 4: Calibrate Limelight Distance

The Limelight's `tx` value needs to be converted to real-world distance.

#### Simple Method (Start Here):

For now, use a placeholder and tune later:
```java
public static final double TX_TO_DISTANCE_FACTOR = 0.15;
```

#### Precise Method:

1. Place robot at known distances from target (1m, 2m, 3m, 4m, 5m)
2. At each distance, record Limelight `tx` value
3. Create a lookup table or calculate a formula

---

### STEP 5: Test in Simulation

1. Press `F5` in VS Code to start simulation
2. Open Elastic or Shuffleboard
3. Look for these values:

| Widget | Purpose |
|--------|---------|
| `Turret/Test_Distance_M` | Set this to test distances (1.0, 2.0, 3.0, etc.) |
| `Turret/Test_Angle_Result` | Shows calculated hood angle |

4. Verify angles are within 49-74° range for your typical shooting distances

**If angles are wrong:**
- Too low (< 49°)? Increase `BALL_VELOCITY_MPS` or decrease `TARGET_HEIGHT_DELTA_M`
- Too high (> 74°)? Decrease `BALL_VELOCITY_MPS` or increase `TARGET_HEIGHT_DELTA_M`

---

### STEP 6: Deploy and Test on Robot

1. **Deploy code:**
   ```
   Right-click on build.gradle → "Deploy Robot Code"
   ```

2. **Test hood movement:**
   - In Driver Station, enable Test mode
   - OR create a button binding in `RobotContainer.java`:
   ```java
   new JoystickButton(controller, Button.kA.value)
       .onTrue(new AutoAimShooter(turret));
   ```

3. **Verify hood angles:**
   - Place robot at 1m, 2m, 3m from target
   - Watch Elastic to see calculated vs actual angles
   - Fine-tune constants

---

## 🎮 Using the System

### Automatic Mode:

```java
// In RobotContainer.java, bind to a button:
new JoystickButton(driverController, Button.kRightBumper.value)
    .whileTrue(new AutoAimShooter(turret));
```

When the button is held:
1. Limelight detects AprilTag
2. System calculates distance from `tx`
3. Solver calculates optimal hood angle
4. Hood moves to angle automatically
5. Shoot when ready!

### Manual Testing:

In Elastic, you can manually test:
1. Set `Turret/Test_Distance_M` to desired distance
2. Run `TestShooterAngle` command
3. Watch hood move to calculated angle

---

## 🔧 Troubleshooting

### Hood doesn't move to correct angle:
- Check `MOTOR_ROTATIONS_PER_DEGREE` is correct
- Verify encoder is working
- Check physical limits aren't being hit

### Angles are way off (too steep or too flat):
- Remeasure `BALL_VELOCITY_MPS` - this is the most common error
- Verify `TARGET_HEIGHT_DELTA_M` is correct
- Try different `IMPACT_ANGLE_RAD` values

### No Limelight data:
- Check Limelight is powered and connected
- Verify NetworkTables connection
- Check `tv` value (should be 1.0 when target visible)

### Shots still miss:
- The formula assumes no air resistance (simplified physics)
- You may need to add empirical corrections
- Create a lookup table for known distances
- Add offset: `calculatedAngle + EMPIRICAL_OFFSET`

---

## 📊 Expected Results

With correct calibration, you should see:

| Distance | Typical Hood Angle |
|----------|-------------------|
| 1.0m     | 55-65° |
| 2.0m     | 50-58° |
| 3.0m     | 52-60° |
| 4.0m     | 55-65° |

*(Actual values depend on your specific robot)*

If your angles are consistently outside 49-74°, your constants need adjustment!

---

## 🚀 Advanced: Add Lookup Table Fallback

For production use, combine physics with empirical data:

```java
// In Turret.java
private double lookupAngle(double distance) {
    // Measured angles at specific distances
    if (distance < 1.5) return 58.0;
    else if (distance < 2.5) return 53.0;
    else if (distance < 3.5) return 55.0;
    else return calculateShooterAngle(); // Use physics
}
```

Good luck! 🎯
