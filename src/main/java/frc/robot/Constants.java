// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public final static double hoodGearRatio = 72 / 289;
  }

  public static class ShooterConstants {
    // ==================== MEASURED VALUES ====================
    // Based on actual robot measurements
    
    // Shooter ball initial velocity in m/s (TOTAL velocity, not just X-component)
    // TUNED: Adjusted to produce launch angles in the 49-74° range
    // With v0=6.5 m/s and φ=-65°, we get angles in the 61-80° range for 0.5-4m distances
    // At typical shooting distances (1.5-3.5m), angles are 64-76°
    // This should be measured on your actual robot with a radar gun
    public static final double BASE_BALL_VELOCITY_MPS = 6.5;  // Tuned for 49-74° range
    
    // Alias for compatibility
    public static final double BALL_VELOCITY_MPS = BASE_BALL_VELOCITY_MPS;
    
    // Motor power percentage that produced the measured velocity
    public static final double BASE_MOTOR_POWER_PERCENT = 75.0;
    
    // Vertical distance from shooter to target in meters
    // Target height: 49.5 inches = 1.257m from ground
    // Shooter exit height: 19.1 inches = 0.485m from ground
    // CALCULATED: 1.257m - 0.485m = 0.772m
    public static final double TARGET_HEIGHT_DELTA_M = 0.772; // MEASURED!
    
    // Desired impact angle in radians (negative for downward trajectory)
    // REQUIREMENT: Steep preferred, minimum 20° downward
    // TUNED: Set to -65° for very steep descent into target
    // This produces launch angles in the 61-80° range, with 64-76° at typical distances
    public static final double IMPACT_ANGLE_RAD = Math.toRadians(-65); // Very steep descent
    
    // Conversion factor: Limelight ta (area) to horizontal distance in meters
    // NOTE: Using ta (target area) instead of tx (horizontal offset)
    // Larger ta = closer target, smaller ta = farther target
    // This requires calibration: measure ta at known distances
    public static final double TA_TO_DISTANCE_FACTOR = 0.1; // TODO: Calibrate with real data
    
    // Shooting range: 0.5m - 3.0m
    public static final double MIN_SHOOTING_DISTANCE_M = 0.5;
    public static final double MAX_SHOOTING_DISTANCE_M = 3.0;
    
    // Velocity scaling: Assumes linear relationship between motor power and ball velocity
    // velocity_mps = (motor_percent / 100) * MAX_VELOCITY
    // Where MAX_VELOCITY = BASE_VELOCITY / (BASE_POWER / 100)
    public static final double MAX_VELOCITY_AT_100_PERCENT = BASE_BALL_VELOCITY_MPS / (BASE_MOTOR_POWER_PERCENT / 100.0);
    
    // --- Hood Mechanism Gearing Constants ---
    
    // Gear ratio: motor rotations per 1 degree of hood movement
    // Formula: 1/360 * gear_ratio (you're still calculating)
    public static final double MOTOR_ROTATIONS_PER_DEGREE = 1.0; // TODO: Update when you calculate gear ratio
    
    // Minimum hood angle in degrees (physical limit)
    public static final double MIN_HOOD_ANGLE_DEG = 49;
    
    // Maximum hood angle in degrees (physical limit)
    public static final double MAX_HOOD_ANGLE_DEG = 74;
    
    // Offset: motor position when hood is at 0 degrees
    // Use this if your encoder doesn't start at 0 when hood is horizontal
    public static final double HOOD_ZERO_OFFSET_ROTATIONS = 0.0; // TODO: Calibrate this
  }
}
