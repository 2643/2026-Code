// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShooterAngleSolver;
import frc.robot.LimelightHelpers;


public class Turret extends SubsystemBase {
  /** Creates a new turretx. */
  TalonFX motorY = new TalonFX(1);
  TalonFX motorX = new TalonFX(4);
  TalonFXConfiguration configs = new TalonFXConfiguration();
  DigitalInput limitX = new DigitalInput(0);
  DigitalInput limitY = new DigitalInput(1);
  public double target = 0;
  public double pos;

  public boolean isVisible;
  public double yaw;
  public double area;
  public double tx;
  public double ty;
  public double fiducialID;
  public double range;
  private final String limelightName = "limelight";
  private final String limelightURL = "http://10.26.43.200:5801/";
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  
  // Limelight NetworkTable
  private NetworkTable limelight;
  private NetworkTableEntry ta; // Target area (changed from tx)
  private NetworkTableEntry ty;
  private NetworkTableEntry tv;

  public Turret() {
    // Get the default Limelight NetworkTable
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ta = limelight.getEntry("ta"); // Using ta (area) instead of tx
    ty = limelight.getEntry("ty");
    tv = limelight.getEntry("tv");
  }
  public void position() {
      var motionmagicconfigs = configs.MotionMagic;
      var slot0configs = configs.Slot0;
  
      slot0configs.kP = 13;
      slot0configs.kI = 0;
      slot0configs.kD = 0;
  
      motionmagicconfigs.MotionMagicAcceleration = 20;
      motionmagicconfigs.MotionMagicCruiseVelocity = 20;
  
      motorX.getConfigurator().apply(configs);
      motorY.getConfigurator().apply(configs);
      motorX.setPosition(0);
      motorY.setPosition(0);
  }
  public String getLimelightURL() {
    return limelightURL;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }
  public void updateData() {
    isVisible = LimelightHelpers.getTV(limelightName);
    yaw = LimelightHelpers.getTX(limelightName);
    tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset (same as yaw)
    ty = LimelightHelpers.getTY(limelightName);  // Vertical offset
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
        
    SmartDashboard.putBoolean("Has Target", isVisible);
    SmartDashboard.putNumber("Target Yaw", yaw);
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Limelight Stream", limelightURL);
  }
  public void autoAlign(){
    if (isVisible == true && tx>0) {
      motorX.setControl(new DutyCycleOut((Math.log(tx)/600*5)));
    } else if (isVisible == true && tx<0) {
      motorX.setControl(new DutyCycleOut(-(Math.log(-tx)/600*5)));
    } 
    else {
      motorX.setControl(new DutyCycleOut(0));
    }
  }
  public void moveToPosY(double target){
    target = pos;
    motorY.setControl(motion.withPosition(target));
  }
  public void moveToPosX(double target){
    target = pos;
    motorX.setControl(motion.withPosition(target));
  }
  public void upMotorPosX(){
    moveToPosX(currentPosX() + 5);
  }
  public void downMotorPosX(){
    moveToPosX(currentPosX() - 5);
  }
  public void upMotorPosY(){
    moveToPosY(currentPosY() + 5);
  }
  public void downMotorPosY(){
    moveToPosY(currentPosY() - 5);
  }
  public double currentPosY(){
    return motorY.getPosition().getValueAsDouble();
  }
  public double currentPosX(){
    return motorX.getPosition().getValueAsDouble();
  }
  public boolean getLimitX(){
    return limitX.get();
  }
  public boolean getLimitY(){
    return limitY.get();
  }

  /**
   * Gets the target area (ta) from the Limelight
   * @return Target area percentage (0-100)
   */
  public double getLimelightTa() {
    return ta.getDouble(0.0);
  }

  /**
   * Gets the vertical offset (ty) from the Limelight
   * @return Vertical offset in degrees
   */
  public double getLimelightTy() {
    return ty.getDouble(0.0);
  }

  /**
   * Checks if the Limelight has a valid target
   * @return true if target detected, false otherwise
   */
  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  /**
   * Calculates the required shooter angle based on Limelight ta value.
   * Uses projectile motion physics to determine the optimal launch angle.
   * 
   * @return Shooter angle in degrees, or 0 if no target is detected
   */
  public double calculateShooterAngle() {
    if (!hasTarget()) {
      return 0.0; // No target, return default angle
    }

    // Get target area from Limelight
    double taValue = getLimelightTa();
    
    // Convert ta to horizontal distance
    double distance = taToDistance(taValue);
    
    // Calculate required velocity for this distance
    double requiredVelocity = calculateRequiredVelocity(distance);
    
    // Use the shooter angle solver
    double thetaRadians = ShooterAngleSolver.solveTheta(
        requiredVelocity,
        distance,
        ShooterConstants.TARGET_HEIGHT_DELTA_M,
        ShooterConstants.IMPACT_ANGLE_RAD
    );
    
    // Convert to degrees for motor control
    return ShooterAngleSolver.toDegrees(thetaRadians);
  }

  /**
   * Converts Limelight ta (target area) to horizontal distance in meters.
   * 
   * @param ta Target area percentage from Limelight
   * @return Distance in meters
   */
  public double taToDistance(double ta) {
    // Simple inverse relationship: larger ta = closer
    // TODO: Calibrate this formula with real measurements at known distances
    double distance = ta * ShooterConstants.TA_TO_DISTANCE_FACTOR;
    
    // Clamp to reasonable shooting range
    return Math.max(ShooterConstants.MIN_SHOOTING_DISTANCE_M,
                    Math.min(ShooterConstants.MAX_SHOOTING_DISTANCE_M, distance));
  }

  /**
   * Calculates the required ball velocity based on distance.
   * For now, uses base velocity. Can be enhanced with distance-based tuning.
   * 
   * @param distance Horizontal distance to target in meters
   * @return Required velocity in m/s
   */
  public double calculateRequiredVelocity(double distance) {
    // Option 1: Fixed velocity (simple)
    // return ShooterConstants.BASE_BALL_VELOCITY_MPS;
    
    // Option 2: Distance-based velocity (more accurate)
    // Closer shots = slower, farther shots = faster
    if (distance < 1.0) {
      return 5.0; // Slow for very close
    } else if (distance < 1.5) {
      return 5.62; // Base speed for mid-close
    } else if (distance < 2.5) {
      return 6.5; // Faster for mid-range
    } else {
      return 7.0; // Fastest for far shots
    }
  }

  /**
   * Converts required velocity (m/s) to motor power percentage.
   * Based on measured relationship: 5.62 m/s at 75% power.
   * 
   * @param velocityMps Required ball velocity in m/s
   * @return Motor power percentage (0-100)
   */
  public double velocityToMotorPercent(double velocityMps) {
    // Linear relationship: velocity is proportional to motor power
    // velocity = (power / 100) * MAX_VELOCITY
    // Therefore: power = (velocity / MAX_VELOCITY) * 100
    
    double motorPercent = (velocityMps / ShooterConstants.MAX_VELOCITY_AT_100_PERCENT) * 100.0;
    
    // Clamp to 0-100%
    motorPercent = Math.max(0.0, Math.min(100.0, motorPercent));
    
    SmartDashboard.putNumber("Turret/Required_Velocity_MPS", velocityMps);
    SmartDashboard.putNumber("Turret/Motor_Power_Percent", motorPercent);
    
    return motorPercent;
  }

  /**
   * Calculates motor power percentage from Limelight ta value.
   * Complete pipeline: ta → distance → velocity → motor percent
   * 
   * @return Motor power percentage (0-100)
   */
  public double calculateMotorPowerFromTA() {
    if (!hasTarget()) {
      return ShooterConstants.BASE_MOTOR_POWER_PERCENT; // Default power
    }
    
    double taValue = getLimelightTa();
    double distance = taToDistance(taValue);
    double requiredVelocity = calculateRequiredVelocity(distance);
    double motorPercent = velocityToMotorPercent(requiredVelocity);
    
    // Publish for debugging
    SmartDashboard.putNumber("Turret/TA_Value", taValue);
    SmartDashboard.putNumber("Turret/Calculated_Distance_M", distance);
    
    return motorPercent;
  }

  /**
   * Converts a hood angle (in degrees) to motor position (in rotations).
   * Takes into account gearing ratio, physical limits, and zero offset.
   * 
   * @param angleDegrees The desired hood angle in degrees
   * @return Motor position in rotations, clamped to physical limits
   */
  public double angleToMotorPosition(double angleDegrees) {
    // Clamp angle to physical limits
    double clampedAngle = Math.max(ShooterConstants.MIN_HOOD_ANGLE_DEG,
                                   Math.min(ShooterConstants.MAX_HOOD_ANGLE_DEG, angleDegrees));
    
    // Convert angle to motor rotations using gear ratio
    double motorRotations = clampedAngle * ShooterConstants.MOTOR_ROTATIONS_PER_DEGREE;
    
    // Apply zero offset
    motorRotations += ShooterConstants.HOOD_ZERO_OFFSET_ROTATIONS;
    
    // Publish for debugging
    SmartDashboard.putNumber("Turret/Hood_Target_Angle_Deg", clampedAngle);
    SmartDashboard.putNumber("Turret/Hood_Motor_Position", motorRotations);
    
    return motorRotations;
  }

  /**
   * Converts a motor position (in rotations) back to hood angle (in degrees).
   * Useful for telemetry and verification.
   * 
   * @param motorRotations The current motor position in rotations
   * @return Hood angle in degrees
   */
  public double motorPositionToAngle(double motorRotations) {
    // Remove zero offset
    double adjustedRotations = motorRotations - ShooterConstants.HOOD_ZERO_OFFSET_ROTATIONS;
    
    // Convert rotations to degrees using gear ratio
    double angleDegrees = adjustedRotations / ShooterConstants.MOTOR_ROTATIONS_PER_DEGREE;
    
    return angleDegrees;
  }

  /**
   * Sets the shooter angle based on Limelight targeting.
   * Call this method to automatically adjust the hood angle.
   * Converts the calculated angle to motor position accounting for gearing.
   */
  public void setShooterAngleFromLimelight() {
    double targetAngleDegrees = calculateShooterAngle();
    double motorPosition = angleToMotorPosition(targetAngleDegrees);
    moveToPosY(motorPosition);
  }

  /**
   * Manually set the hood to a specific angle (in degrees).
   * Converts to motor position automatically.
   * 
   * @param angleDegrees Desired hood angle in degrees
   */
  public void setHoodAngle(double angleDegrees) {
    double motorPosition = angleToMotorPosition(angleDegrees);
    moveToPosY(motorPosition);
  }

  /**
   * Gets the current hood angle in degrees.
   * Converts from motor position.
   * 
   * @return Current hood angle in degrees
   */
  public double getCurrentHoodAngle() {
    double motorPos = currentPosY();
    return motorPositionToAngle(motorPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
