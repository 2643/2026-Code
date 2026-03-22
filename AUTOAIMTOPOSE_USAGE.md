# AutoAimToPose Usage Guide

## What is AutoAimToPose?

`AutoAimToPose` is a Command that automatically aims your turret at a target location using the robot's odometry pose. Instead of using Limelight vision to track a target, it:

1. Takes a `Pose2d` (position + rotation) as a target
2. Calls `Swivel.turretTrackPose()` to calculate the correct turret angle
3. Continuously updates turret position until interrupted

**Use this when:** You know the exact target location beforehand (e.g., hub, speaker, scoring location)  
**Don't use this when:** You're tracking a moving target (use `AutoAim` for that)

---

## Where AutoAimToPose Should Be Called

### Location 1: PathPlanner NamedCommands (For Autonomous)

This is the **BEST place** to use `AutoAimToPose`. Your PathPlanner autos can schedule turret aiming.

**File to edit:** `src/main/java/frc/robot/RobotContainer.java`

Look for the `NamedCommands` section in the constructor (around line 117):

```java
public RobotContainer() {
    NamedCommands.registerCommand("Intake", new StartIntake(true));
    NamedCommands.registerCommand("Shoot", new ToggleWheel(Phase.ATTACK));
    NamedCommands.registerCommand("ManualTurret", new ManualTurret());
    NamedCommands.registerCommand("ResetHood", new ResetHood());
    NamedCommands.registerCommand("ResetSwivel", new ResetSwivel());
    
    // 🔴 ADD THIS:
    NamedCommands.registerCommand("AutoAimHub", 
        new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d())));
    
    NamedCommands.registerCommand("AutoAimSpeaker",
        new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d())));
```

Then in your PathPlanner `.auto` files, add a step:

```json
{
  "type": "named",
  "data": {
    "name": "AutoAimHub"
  }
}
```

### Location 2: Teleop with Button Press

You could add a button in teleop that enables aim-at-hub:

**File to edit:** `src/main/java/frc/robot/RobotContainer.java`

In the `configureBindings()` method (around line 167):

```java
private void configureBindings() {
    toggle.onTrue(new Toggle());
    intake.onTrue(new StartIntake());
    shoot.onTrue(new ToggleWheel(m_Storage.getPhase()));
    
    // 🔴 ADD THIS:
    // Press button X to aim at hub for 2 seconds
    Trigger aimButton = /* your button here */;
    aimButton.onTrue(
        new AutoAimToPose(
            new Pose2d(Constants.TurretConstants.hubX, 
                      Constants.TurretConstants.hubY, 
                      new Rotation2d()))
        .withTimeout(2.0)  // Stop after 2 seconds
    );
    
    // Or use the Pose2d directly:
    // aimButton.whileTrue(new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d())));
}
```

### Location 3: Custom Autonomous Command

If you want to write a custom autonomous routine (not PathPlanner):

```java
public Command getAutonomousCommand() {
    return Commands.sequence(
        // Drive to position
        new PathPlannerAuto("S1-O-Shoot"),
        
        // 🔴 ADD THIS: Aim at hub while moving
        new AutoAimToPose(
            new Pose2d(Constants.TurretConstants.hubX, 
                      Constants.TurretConstants.hubY,
                      new Rotation2d())
        )
        .withTimeout(1.0),
        
        // Shoot
        new Shoot(Phase.ATTACK)
    );
}
```

---

## How to Use AutoAimToPose

### Basic Usage

```java
// Aim at hub indefinitely (until interrupted)
new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d()))

// Aim for 2 seconds, then stop
new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d()))
    .withTimeout(2.0)

// Aim while button is held
button.whileTrue(new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d())))
```

### Using Constants

**BEST PRACTICE:** Use constants instead of hardcoding coordinates:

```java
// ✅ GOOD - Uses Constants.java values
new AutoAimToPose(
    new Pose2d(Constants.TurretConstants.hubX, 
               Constants.TurretConstants.hubY,
               new Rotation2d())
)

// ❌ BAD - Hardcoded values
new AutoAimToPose(new Pose2d(8.23, 4.11, new Rotation2d()))
```

### Different Target Locations

If you have multiple scoring locations:

```java
// In Constants.java, add:
public static final double speakerX = 8.23;
public static final double speakerY = 4.11;
public static final double ampX = 1.84;
public static final double ampY = 7.70;

// Then use in commands:
NamedCommands.registerCommand("AimSpeaker",
    new AutoAimToPose(new Pose2d(speakerX, speakerY, new Rotation2d())));

NamedCommands.registerCommand("AimAmp",
    new AutoAimToPose(new Pose2d(ampX, ampY, new Rotation2d())));
```

---

## Complete Example: Adding to RobotContainer

Here's what your RobotContainer should look like after adding `AutoAimToPose`:

```java
public RobotContainer() {
    // ===== EXISTING COMMANDS =====
    NamedCommands.registerCommand("Intake", new StartIntake(true));
    NamedCommands.registerCommand("Shoot", new ToggleWheel(Phase.ATTACK));
    NamedCommands.registerCommand("ManualTurret", new ManualTurret());
    NamedCommands.registerCommand("ResetHood", new ResetHood());
    NamedCommands.registerCommand("ResetSwivel", new ResetSwivel());
    NamedCommands.registerCommand("ToggleWheel", new ToggleWheel(m_Storage.getPhase()));
    NamedCommands.registerCommand("ToggleIndexer", new ToggleIndexer(true));

    // ===== NEW: AUTO-AIM COMMANDS =====
    NamedCommands.registerCommand("AutoAimHub",
        new AutoAimToPose(
            new Pose2d(Constants.TurretConstants.hubX,
                      Constants.TurretConstants.hubY,
                      new Rotation2d())
        )
    );

    configureBindings();

    // Existing auto chooser...
    autoChooser.addOption("S1 Shoot", new PathPlannerAuto("S1-O-Shoot"));
    autoChooser.addOption("S2 Shoot", new PathPlannerAuto("S2-MID-Shoot"));
}
```

---

## How It Works

### Behind the Scenes

When you create `AutoAimToPose`:

```
1. Initialize with target Pose2d
   ↓
2. On execute() call every 20ms:
   ├─ Get current robot pose
   ├─ Call turretTrackPose(target)
   ├─ Calculate robot-relative angle
   ├─ Command swivel motor to new angle
   ↓
3. Continue until interrupted or timeout
```

### SmartDashboard During Execution

You'll see these update in real-time:

```
Robot Odometry:
├─ Robot X: 3.544m  (actual position)
├─ Robot Y: 4.025m
└─ Robot Rotation: 180°

Turret Aiming:
├─ Angle to Hub (Field Rel): 53.2°
├─ Angle to Hub (Robot Rel): -126.8°
├─ Distance to Hub: 5.8m
├─ Current Swivel Pos: -0.35 rotations
└─ Target Swivel Pos: -0.35 rotations
```

---

## Integration with PathPlanner Autos

### Example: S1-O-Shoot Auto

Your auto file (`src/main/deploy/pathplanner/autos/S1-O-Shoot.auto`) should have this structure:

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "named",
          "data": {
            "name": "ResetHood"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "ResetSwivel"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "S1-O"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "AutoAimHub"    // 🔴 ADD THIS
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Shoot"
          }
        }
      ]
    }
  }
}
```

**This sequence:**
1. Reset hood (Level it)
2. Reset swivel (Center it)
3. Drive to intake position (Path: S1-O)
4. **Aim at hub** (AutoAimToPose - NEW!)
5. Shoot

---

## Teleop Integration

If you want a button to aim at hub in teleop:

```java
private void configureBindings() {
    // Existing bindings...
    drivetrain.setDefaultCommand(/* ... */);
    
    // 🔴 ADD THIS: Use one of your available buttons
    // Example: Use rightBumper to aim at hub
    CommandJoystick rightJoystick = new CommandJoystick(1);
    Trigger aimButton = rightJoystick.button(10);
    
    aimButton.whileTrue(
        new AutoAimToPose(
            new Pose2d(Constants.TurretConstants.hubX,
                      Constants.TurretConstants.hubY,
                      new Rotation2d())
        )
    );
}
```

---

## Differences: AutoAimToPose vs AutoAim vs Manual Aiming

| Feature | AutoAimToPose | AutoAim | Manual |
|---------|---------------|---------|--------|
| **What it tracks** | Fixed target pose | Limelight vision | Joystick input |
| **Requires vision** | ❌ No | ✅ Yes | ❌ No |
| **Use case** | Autonomous, known targets | Teleop, moving targets | Fine adjustment |
| **Accuracy** | ±0.2m from odometry | ±0.1m with tags | Manual |
| **File** | `AutoAimToPose.java` | `AutoAim.java` | `ManualTurret.java` |
| **Best for** | PathPlanner autos | Finding targets | Last-second tweaks |

---

## Required Imports

Make sure your file imports `AutoAimToPose`:

```java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.Turret.AutoAimToPose;
import frc.robot.Constants;
```

---

## Testing AutoAimToPose

### Bench Test (Robot Not Moving)

1. Place robot on stand (wheels off ground)
2. Enable teleop
3. Press your aim button
4. Check SmartDashboard:
   - `Angle to Hub` should be calculated
   - `Current Swivel Position` should move to match target
   - No errors in console

### Gym Test (Robot Moving)

1. Place robot at known position (e.g., 0,0)
2. Enable autonomous or press aim button
3. Robot should calculate turret angle to hub
4. Move robot around, watch angle update correctly
5. Verify turret follows calculated angle

### With PathPlanner Auto

1. Select auto with `AutoAimHub` step
2. Robot drives to position
3. At `AutoAimHub` step, turret rotates to aim
4. Then `Shoot` command fires

---

## Common Issues & Fixes

| Problem | Cause | Fix |
|---------|-------|-----|
| Turret doesn't move when `AutoAimToPose` runs | Swivel not initialized or wrong mode | Check Swivel state is INITIALIZED and mode is AUTOAIM |
| Turret aims wrong direction | Wrong hub coordinates | Update Constants: `hubX` and `hubY` |
| Compilation error: "Cannot find symbol" | Missing import | Add `import frc.robot.commands.Turret.AutoAimToPose;` |
| Turret oscillates/shakes | PID gains too aggressive | Lower `swivelP` in Constants from 20 to 10-15 |
| Angle calculation seems off | Robot starting pose wrong | Verify starting pose in PathPlanner paths (START3 waypoint) |

---

## Summary: Where to Add AutoAimToPose

### Quick Checklist

- [ ] **Option 1: PathPlanner Autos (Recommended)**
  - Add NamedCommand in `RobotContainer.constructor()`
  - Add "AutoAimHub" step in `.auto` files
  
- [ ] **Option 2: Teleop Button**
  - Add button binding in `configureBindings()`
  - Use `.whileTrue()` or `.onTrue().withTimeout()`
  
- [ ] **Option 3: Custom Auto Command**
  - Use in `Commands.sequence()` or `Commands.parallel()`
  - Call from `getAutonomousCommand()`

### Files to Modify

1. **DEFINITELY:** `RobotContainer.java` - Add NamedCommand
2. **MAYBE:** `.auto` files - Add step to execute aiming
3. **MAYBE:** `Constants.java` - Add speaker/amp positions if using multiple targets
4. **PROBABLY NOT:** Other files - System is self-contained

---

## Next Steps

1. **Add NamedCommand** in `RobotContainer.java`:
   ```java
   NamedCommands.registerCommand("AutoAimHub",
       new AutoAimToPose(new Pose2d(Constants.TurretConstants.hubX,
                                    Constants.TurretConstants.hubY,
                                    new Rotation2d())));
   ```

2. **Update your `.auto` files** to include the "AutoAimHub" step where needed

3. **Test in simulation** first:
   ```bash
   ./gradlew simulateJava
   ```

4. **Test on robot** in gym (bench test first, then with motion)

5. **Verify SmartDashboard** shows correct aiming angles

---

**Remember:** `AutoAimToPose` uses **odometry + known target position** to calculate aim.  
It's perfect for autonomous when you know exactly where the hub/speaker is!
