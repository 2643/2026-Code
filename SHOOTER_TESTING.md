# Shooter Angle Testing Guide

## Testing WITHOUT a Physical Robot

You can test the shooter angle calculation system using WPILib simulation and Elastic/Shuffleboard!

### Method 1: Using WPILib Simulation + Elastic Dashboard

#### Step 1: Start Simulation
1. In VS Code, press `Ctrl+Shift+P`
2. Type "WPILib: Simulate Robot Code"
3. Select your robot project
4. Click "Sim GUI" when it opens

#### Step 2: Open Elastic Dashboard
1. Download Elastic from: https://github.com/Gold872/elastic-dashboard
2. Run Elastic and connect to `localhost:5810` (default NetworkTables port)
3. Or use Shuffleboard (comes with WPILib)

#### Step 3: Test the Math
In Elastic/Shuffleboard, you'll see these values:

**Input Values (you can change these):**
- `Turret/Test_Distance_M` - Set this to different distances (1-10 meters)
- `Turret/Test_TX_Input` - Simulate Limelight TX values

**Output Values (calculated automatically):**
- `Turret/Calculated_Angle_Deg` - The angle the solver calculated
- `Turret/Test_Angle_Result` - Result from test function

**Debug Values:**
- `Turret/HasTarget` - Simulated target detection
- `Turret/Limelight_TX` - Current TX value
- `Turret/Current_Pos_X/Y` - Motor positions

#### Step 4: Simulate Limelight Data
To simulate the Limelight sending data:

1. Open **Outline View** in Shuffleboard/Elastic
2. Navigate to `limelight` table
3. Set these values:
   - `tx` = horizontal offset (try values like -10, 0, 10)
   - `ty` = vertical offset 
   - `tv` = 1 (means target found)

### Method 2: Run Test Suite

Add this to your `Robot.java` or `RobotContainer.java`:

```java
import frc.robot.util.ShooterAngleTester;

// In robotInit() or test mode:
@Override
public void testInit() {
    ShooterAngleTester.runTestSuite();
}
```

This will test angles for distances 1-10 meters and publish results to SmartDashboard.

### Method 3: Manual Testing in Code

You can also test directly in your code:

```java
import frc.robot.util.ShooterAngleTester;

// Test a specific distance
ShooterAngleTester.testSingleDistance(5.0); // Test at 5 meters

// Simulate Limelight behavior
ShooterAngleTester.simulateLimelightMapping();
```

## Viewing Results in Elastic

### Recommended Dashboard Layout:

1. **Distance Input** - Number input widget for `Turret/Test_Distance_M`
2. **Angle Output** - Number display for `Turret/Test_Angle_Result`
3. **Limelight Status** - Boolean for `Turret/HasTarget`
4. **Graph Widget** - Plot angle vs distance to visualize the curve

### Expected Results

For reasonable values:
- **Distance: 5m** → Angle: ~30-50° (depends on your constants)
- **Distance: 10m** → Angle: ~40-60°

If you get weird results (like 90° or 0°), check your constants in `Constants.java`:
- `BALL_VELOCITY_MPS` - Should be realistic (8-15 m/s typical)
- `TARGET_HEIGHT_DELTA_M` - Height difference shooter to target
- `IMPACT_ANGLE_RAD` - Usually negative for downward trajectory

## Tuning Constants

### 1. Physics Constants (Test in Simulation First)

1. Start with realistic physics:
   - Measure your shooter's ball velocity (use a radar gun or time over distance)
   - Measure height from shooter to target
   - Choose impact angle (typically -30 to -60 degrees)

2. Test in simulation first
3. Adjust `TX_TO_DISTANCE_FACTOR` based on your camera specs

### 2. Gearing Constants (Requires Physical Robot)

The hood angle needs to be converted to motor position. Here's how to calibrate:

#### Step 1: Measure Gear Ratio

**Option A - Direct Measurement:**
1. Manually move hood from 0° to a known angle (e.g., 45°)
2. Read motor encoder value
3. Calculate: `MOTOR_ROTATIONS_PER_DEGREE = encoder_value / 45.0`

**Option B - Gear Calculation:**
```
MOTOR_ROTATIONS_PER_DEGREE = (motor_gear_teeth / hood_gear_teeth) / 360.0
```

Example:
- Motor gear: 12 teeth
- Hood gear: 72 teeth  
- Ratio: 12/72 = 1/6
- One motor rotation = 6° of hood movement
- `MOTOR_ROTATIONS_PER_DEGREE = 1/6 / 1 = 0.1667`

#### Step 2: Find Physical Limits

1. Manually move hood to lowest position → Note encoder value → `MIN_HOOD_ANGLE_DEG`
2. Manually move hood to highest position → Note encoder value → `MAX_HOOD_ANGLE_DEG`

#### Step 3: Calibrate Zero Offset

1. Move hood to a known angle (e.g., perfectly horizontal = 0°)
2. Read motor encoder value
3. Set `HOOD_ZERO_OFFSET_ROTATIONS = -(encoder_value)`

#### Step 4: Test with TestHoodGearing Command

In Elastic/SmartDashboard:
1. Set `Turret/Test_Set_Angle_Deg` to 30
2. Run `TestHoodGearing` command
3. Verify hood actually moves to 30°
4. If not, adjust `MOTOR_ROTATIONS_PER_DEGREE`
5. Repeat with different angles (0°, 15°, 45°, 60°)

### 3. Limelight Distance Calibration

4. Deploy to real robot and fine-tune

## Troubleshooting

**No values showing in Elastic?**
- Make sure robot code is running (sim or real)
- Check NetworkTables connection (should be green)
- Verify SmartDashboard.putNumber() is being called in periodic()

**Angles look wrong?**
- Check your constants (especially velocity and height)
- Verify units (meters, not feet!)
- Test with known distances

**Want to see the math step-by-step?**
- Add print statements in `ShooterAngleSolver.solveTheta()`
- Watch the binary search converge to the solution

## Advanced: Custom Distance Function

If you want a more accurate tx-to-distance conversion, replace the simple factor with:

```java
// In Turret.java
public double txToDistance(double tx) {
    // Use camera specs and trigonometry
    double cameraHeight = 0.5; // meters
    double targetHeight = 2.5; // meters  
    double mountAngle = Math.toRadians(30); // camera tilt
    
    double targetAngle = mountAngle + Math.toRadians(tx);
    double distance = (targetHeight - cameraHeight) / Math.tan(targetAngle);
    
    return distance;
}
```

Then use it in `calculateShooterAngle()`.

---

Happy testing! 🚀
