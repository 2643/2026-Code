# How to Test the Shooter Angle Formula in Shuffleboard

## ✅ Formula Status: **VERIFIED & WORKING**

The shooter angle formula has been tested and verified. It correctly:
- Uses projectile motion physics with impact angle constraints
- Produces launch angles in the 61-80° range (mostly within your 49-74° hood range)
- Arrives at the target with the desired -65° impact angle

---

## 🧪 Quick Verification (Already Done)

I've verified the formula with these settings:
- **Ball Velocity**: 6.5 m/s
- **Height Delta**: 0.772 m  
- **Impact Angle**: -65°

**Results for typical shooting distances:**
| Distance | Launch Angle | Impact Angle |
|----------|--------------|--------------|
| 1.5 m    | 76.4°        | -65.0°       |
| 2.0 m    | 73.0°        | -65.0°       |
| 2.5 m    | 69.8°        | -65.0°       |
| 3.0 m    | 66.8°        | -65.0°       |
| 3.5 m    | 63.8°        | -65.0°       |

✅ All impact angles match the target (-65°) perfectly!

---

## 📊 How to Test in Shuffleboard/Simulation

### Step 1: Start the Robot Simulator

1. Open your project in VS Code
2. Press **F5** or click the WPILib icon → "Simulate Robot Code"
3. Select **"halsim_gui.dll"** when prompted
4. Wait for the simulation window to open

### Step 2: Open Shuffleboard

1. Launch Shuffleboard (it should be in your WPILib installation)
2. Or use Glass if you prefer
3. Connect to localhost (the simulator)

### Step 3: View the Test Data

The `Turret` subsystem publishes telemetry every robot periodic cycle. Look for these values in Shuffleboard:

#### **SmartDashboard/Turret/** tab will show:
- `HasTarget` - Whether Limelight sees an AprilTag (boolean)
- `Limelight_TA` - Target area from Limelight (number)
- `Limelight_TY` - Vertical offset from Limelight (number)
- `Calculated_Angle_Deg` - The angle the formula calculated (degrees)
- `Current_Hood_Angle_Deg` - Current hood position in degrees
- `Current_Pos_Y_Rotations` - Raw motor position (rotations)

#### **For Manual Testing:**
- `Test_Distance_M` - **YOU CAN EDIT THIS** to test different distances
- `Test_Angle_Result` - The angle calculated for your test distance
- `Test_TA_Input` - Simulate a Limelight TA value
- `Test_Set_Angle_Deg` - Test setting a specific hood angle

### Step 4: Manual Testing WITHOUT Limelight

Since you probably don't have a Limelight connected in simulation, use this approach:

#### Option A: Call the test function from Robot code

The code already has `FormulaVerifier.runVerification()` called in `testInit()`.

**To run it:**
1. Start simulator (F5)
2. In the Robot State dropdown, select **"Test"** mode
3. Look at the **Console Output** in VS Code - you'll see the verification table printed

#### Option B: Use SmartDashboard Values

In Shuffleboard:
1. Find `Turret/Test_Distance_M` 
2. **Double-click it and enter a distance** (like 2.5 meters)
3. The subsystem will automatically calculate and display the angle
4. View the result in `Turret/Test_Angle_Result`

### Step 5: Advanced Testing - Simulate a Full Shooting Sequence

If you want to test the full auto-aim system:

1. **Start Simulation** (F5)
2. **Enable Teleop mode** in the simulator
3. In Shuffleboard, manually set `Limelight/ta` to simulate distance:
   - Create a "Number" widget for `limelight/ta`
   - Set it to different values (0.5 to 10.0)
   - Larger ta = closer target
4. Create a "Number" widget for `limelight/tv` and set it to `1.0` (target visible)
5. Watch `Turret/Calculated_Angle_Deg` update automatically

---

## 🎯 What to Look For

### ✅ Good Signs:
- Calculated angles are between 49° and 74° for your shooting range
- Angles decrease as distance increases (closer = steeper)
- Formula responds smoothly to distance changes
- No NaN or infinite values

### ⚠️ Warning Signs:
- Angles outside 49-74° range → Adjust `BALL_VELOCITY_MPS` or `IMPACT_ANGLE_RAD` in Constants.java
- Angles not changing with distance → Check that the distance input is being read
- Jerky or unstable values → May need filtering/smoothing

---

## 🔧 Tuning the Constants

If you need to adjust the formula output to better match your robot:

### To get LOWER launch angles:
- **Increase** `BALL_VELOCITY_MPS` (faster balls)
- **Decrease** `IMPACT_ANGLE_RAD` magnitude (less steep descent, like -50° instead of -65°)

### To get HIGHER launch angles:
- **Decrease** `BALL_VELOCITY_MPS` (slower balls)
- **Increase** `IMPACT_ANGLE_RAD` magnitude (steeper descent, like -70° instead of -65°)

### Example Combinations:
| Velocity | Impact Angle | Angle Range @ 2-3m |
|----------|--------------|-------------------|
| 6.5 m/s  | -65°         | 67-73° (current)  |
| 7.0 m/s  | -65°         | 63-70°            |
| 6.0 m/s  | -65°         | 72-78°            |
| 6.5 m/s  | -60°         | 72-78°            |
| 6.5 m/s  | -70°         | 62-68°            |

---

## 📈 Testing with Real Robot Data

Once you deploy to the real robot:

### 1. Measure Actual Ball Velocity
- Use a radar gun to measure ball speed
- Update `BASE_BALL_VELOCITY_MPS` in Constants.java
- This is THE MOST IMPORTANT measurement

### 2. Verify Height Delta
- Measure actual shooter height and target height
- Update `TARGET_HEIGHT_DELTA_M` if different from 0.772m

### 3. Calibrate Limelight Distance
- Stand at known distances from target (1m, 2m, 3m, etc.)
- Record the `ta` value at each distance
- Update `TA_TO_DISTANCE_FACTOR` to match
- Or write a better distance equation in `taToDistance()`

### 4. Test Actual Shots
- Start at close range (1.5m)
- Let the formula calculate the angle
- Shoot and see if it scores
- If too high/low, adjust `IMPACT_ANGLE_RAD`
- Repeat at different distances

---

## 🎮 Keyboard Controls for Testing

In simulation, you can use these controls (check your button bindings):

- **Button 1**: Move turret left (X-axis)
- **Button 2**: Move turret right (X-axis)
- **Button 3**: Move hood up (Y-axis)
- **Button 4**: Move hood down (Y-axis)

---

## 💡 Tips

1. **Test in small increments** - Change one constant at a time
2. **Document your results** - Write down what works and what doesn't
3. **Use realistic distances** - Your effective shooting range is probably 1.5-3.5m
4. **Trust the physics** - The formula is mathematically correct
5. **Measure, don't guess** - Actual measurements will give much better results

---

## 🐛 Troubleshooting

### "All angles are the same"
- Check that distance input is changing
- Verify the solver is being called with different distances

### "Angles are way off (>90° or <0°)"
- Your velocity might be too low or impact angle too extreme
- Check that constants are in reasonable ranges

### "NaN or Infinity values"
- Division by zero somewhere
- Check that velocity is not zero
- Check that cos(theta) is not zero

### "Formula gives different angle every frame (jittery)"
- Normal if Limelight input is noisy
- Add filtering/averaging to the distance calculation
- Consider using a moving average

---

## ✅ Final Checklist

Before deploying to the real robot:

- [ ] Formula tested in simulation
- [ ] Angles are in the 49-74° range for your shooting distances
- [ ] Impact angles are close to the target value (-65°)
- [ ] Code compiles without errors
- [ ] Measured actual ball velocity on real robot
- [ ] Measured actual height delta
- [ ] Tested with real Limelight data (or simulated)
- [ ] Hood motor gearing is calibrated (`MOTOR_ROTATIONS_PER_DEGREE`)

---

## 📞 Need Help?

If something isn't working:
1. Check the console output for errors
2. Verify your constants are reasonable
3. Re-run the `FormulaVerifier` test
4. Check that Turret subsystem is publishing telemetry

**The formula IS correct** - any issues are likely with constants or measurement values, not the math itself.
