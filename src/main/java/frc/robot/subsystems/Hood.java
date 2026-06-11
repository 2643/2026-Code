// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.subsystems.Swivel.States;
import frc.robot.util.LimelightHelpers;

public class Hood extends SubsystemBase {
  public boolean disable = false;
  public boolean isLocked = false;
  public boolean isVisible;
  public double yaw;
  public double area;
  public double ty;
  public double fiducialID;
  public double range;
  public double roundedArea;
  public static double hoodTarget;
  public double angle = -1;
  private Timer timer = new Timer();
  public boolean reset = false;
  private boolean homingInProgress = false;
  private double homingStartTime = 0.0;
  public double slope = 1.3869;
  public double offset = 1.13255;

  public SparkMax hoodMotor = new SparkMax(Constants.TurretConstants.hoodID, MotorType.kBrushless);
  public RelativeEncoder encoder = hoodMotor.getEncoder();
 
  // private final String limelightName2 = "limelight";
  // private final String limelightURL2 = "http://10.26.43.200:5801/";
   private final String limelightName = "limelight-bhavik";
  private final String limelightURL = "http://10.26.43.201:5801/";

  // public DigitalInput hoodLimit = new DigitalInput(Constants.TurretConstants.hoodLimitPort); //removed bc different initialization method

  TalonFXConfiguration configs = new TalonFXConfiguration();
  
  public MAXMotionConfig motorConfig = new MAXMotionConfig();
  public ClosedLoopConfig motorConfigClosed = new ClosedLoopConfig();
  public SparkMaxConfig motorConfigBase = new SparkMaxConfig();
  ClosedLoopConfig revConfig = new ClosedLoopConfig();
  SparkClosedLoopController m_controller = hoodMotor.getClosedLoopController();

  


  public Hood() {
    motorConfig.cruiseVelocity(Constants.TurretConstants.hoodVel).maxAcceleration(Constants.TurretConstants.hoodAccel);

    motorConfigClosed
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.TurretConstants.hoodP)
      .i(Constants.TurretConstants.hoodI)
      .d(Constants.TurretConstants.hoodD)
      .outputRange(-5, 5)
      .apply(motorConfig);

      motorConfigBase.apply(motorConfigClosed);

    hoodMotor.configure(motorConfigBase, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  
  public void moveHood(double position) {
      // position is a logical hood angle/position (from lookup or heuristic).
      // Convert to encoder units using live scale/offset so we can tune mapping.
      double scale = SmartDashboard.getNumber("Hood/AngleToEncoderScale", 1.0);
      double off = SmartDashboard.getNumber("Hood/AngleToEncoderOffset", 0.0);
      SmartDashboard.putNumber("Hood/AngleToEncoderScale", scale);
      SmartDashboard.putNumber("Hood/AngleToEncoderOffset", off);

      double encodedSetpoint = position * scale + off;
      hoodTarget = encodedSetpoint; // store encoder-domain target so isAtPosition works
      SmartDashboard.putNumber("Hood/RequestedLogical", position);
      SmartDashboard.putNumber("Hood/RequestedEncoded", encodedSetpoint);
      SmartDashboard.putNumber("Hood/CurrentEncoded", encoder.getPosition());

      // Command the SparkMax closed-loop controller. If the MAXMotion control
      // mode doesn't move as expected, you can switch to a basic position control.
      try {
        m_controller.setSetpoint(encodedSetpoint, ControlType.kPosition);
      } catch (Throwable t) {
        // Fallback to simple position control
        try {
          m_controller.setSetpoint(encodedSetpoint, ControlType.kPosition);
        } catch (Throwable t2) {
          SmartDashboard.putString("Hood/SetpointErr", t2.toString());
        }
      }
  }
  public double getHoodPos() {
    return encoder.getPosition();
  }

  public boolean isAtPosition() {
    double encoderPosition = encoder.getPosition();
    return Math.abs(encoderPosition - hoodTarget) <= 1; // MARGIN OF ERROR
  }

  public double getTY() {
    return ty;
  }
  
  public void autoPitch() {
    // Use fused robot pose to compute turret-to-hub distance and lookup hood angle
    if (RobotContainer.m_Storage.getPhase() == Phase.ATTACK) {
  boolean usedFused = false;
      double lookupAngle = Double.NaN;
      double clamped = Double.NaN;
      double dist = Double.NaN;
  // Live adjustment (meters/degrees depending on units in lookup) to raise/lower hood
  double hoodAdjust = SmartDashboard.getNumber("Hood/AngleAdjust", 0.0);
  SmartDashboard.putNumber("Hood/AngleAdjust", hoodAdjust);
      try {
        if (RobotContainer.drivetrain != null && RobotContainer.drivetrain.getState() != null && RobotContainer.drivetrain.getState().Pose != null) {
          var robotPose = RobotContainer.drivetrain.getState().Pose;
          // treat a default Pose2d (0,0,0) as possibly uninitialized
          if (!(robotPose.getTranslation().getX() == 0.0 && robotPose.getTranslation().getY() == 0.0 && robotPose.getRotation().getDegrees() == 0.0)) {
            dist = frc.robot.util.TurretUtil.getDistance(robotPose, frc.robot.util.TurretUtil.TargetType.HUB);
            lookupAngle = frc.robot.util.TurretUtil.getTrajectoryAngle(dist, frc.robot.util.TurretUtil.TargetType.HUB);
            // Apply live adjust. Only clamp the upper bound (soft max). Allow
            // values below the previous soft minimum so AutoAim can choose
            // lower hood angles when needed.
            lookupAngle += hoodAdjust;
            clamped = Math.min(Constants.TurretConstants.hoodSoftLimit1, lookupAngle);
            angle = clamped;
            moveHood(clamped);
            usedFused = true;
          }
        }
      } catch (Throwable t) {
        SmartDashboard.putString("Hood/Error", t.toString());
      }

      // If we didn't use the fused pose (no pose yet), fall back to area heuristic
      if (!usedFused) {
        roundedArea = (area > 0) ? Math.log(1.0 / area) : roundedArea;
        angle = (slope * roundedArea) - offset + hoodAdjust;
        // Only enforce the upper soft limit. Allow the hood to move below the
        // previous lower bound so there's no minimum enforced by software.
        if (angle < Constants.TurretConstants.hoodSoftLimit1) {
          moveHood(angle);
        }
      }

      // Diagnostics for tuning and debugging
      SmartDashboard.putBoolean("Hood/UsingFusedPose", usedFused);
      SmartDashboard.putNumber("Hood/LookupAngle", Double.isNaN(lookupAngle) ? -1 : lookupAngle);
      SmartDashboard.putNumber("Hood/Clamped", Double.isNaN(clamped) ? -1 : clamped);
      SmartDashboard.putNumber("Hood/DistanceToHub", Double.isNaN(dist) ? -1 : dist);
    } else {
      moveHood(2.7);
    }
  }


  public void setEncoder() {
    // Start a non-blocking homing routine: drive to the known soft-max
    // and then record encoder position as that known value. This avoids
    // setting encoder arbitrarily while hood is somewhere unknown.
    // Zero the encoder at the current physical position first so "0" maps
    // to where the hood physically is now. Then command a closed-loop move
    // to the soft limit (homing target).
    encoder.setPosition(0.0);
    reset = false;
    homingInProgress = true;
    homingStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    // Command to move to the soft limit (max hood)
    m_controller.setSetpoint(0.1, ControlType.kDutyCycle);
    // moveHood(Constants.TurretConstants.hoodSoftLimit1);
  }

//hello sigmas, this is joshua, I am now in the code mwahhahahahh (signed 3/9/2026)
  /**
   * Direct percent output to hood motor. Interprets input as a percent
   * (range -1.0 .. 1.0). This is intended for manual control/testing only.
   * Do NOT pass encoder setpoints here.
   */
  public void setHood(double percent) {
    // Clamp to safe range to avoid accidental full-speed commands.
    double out = Math.max(-1.0, Math.min(1.0, percent));
    hoodMotor.set(out);
  }

  public void startTimer() {
      timer.start();
  }
  
  public void resetTimer() {
    timer.stop();
    timer.reset();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);

    // if(RobotContainer.m_Swivel.getState() == States.INITIALIZED) {
    //  if (getHoodPos() > Constants.TurretConstants.hoodSoftLimit1) {
    //   moveHood(Constants.TurretConstants.hoodSoftLimit1 - 0.1);
    // } else if (getHoodPos() < Constants.TurretConstants.hoodSoftLimit2) {
    //   moveHood(Constants.TurretConstants.hoodSoftLimit2 + 0.1);
    // } else if (getHoodPos() >= Constants.TurretConstants.hoodHardLimit1 || getHoodPos() <= Constants.TurretConstants.hoodHardLimit2) {
    //   // disable = true;
    // }
    if (encoder.getPosition() >= Constants.TurretConstants.hoodHardLimit1*2) {
      m_controller.setSetpoint(0, ControlType.kDutyCycle);
      encoder.setPosition(Constants.TurretConstants.hoodSoftLimit1);
    }
    // }
    
   if (timer.hasElapsed(1) && reset == false){
    // moveHood(SmartDashboard.getNumber("Target Hood Position", hoodTarget));
    slope = SmartDashboard.getNumber("Slope", 1.3869);
    offset = SmartDashboard.getNumber("Offset", 1.13255);
    moveHood(1);
    reset = true;
  }
  
  // Auto-pitch only when turret is initialized and in attack phase
  if (RobotContainer.m_Swivel.getState() == States.INITIALIZED && timer.hasElapsed(3) && reset) {
    autoPitch();
  }
    
    SmartDashboard.putNumber("Target Hood Position", hoodTarget);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Driver Cam", limelightURL);
    SmartDashboard.putNumber("Current Hood Position", getHoodPos());
    SmartDashboard.putNumber("Rounded Area", roundedArea);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Offset", offset);
    SmartDashboard.putNumber("Slope", slope);
    SmartDashboard.putBoolean("Hood/HomingInProgress", homingInProgress);
    SmartDashboard.putNumber("Hood/HomingStart", homingStartTime);

    // Complete non-blocking homing: if in progress, wait until position is reached or timeout
    if (homingInProgress) {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean reached = isAtPosition();
      // timeout after 3 seconds
      if (reached || (now - homingStartTime) > 3.0) {
        // Set encoder so current position equals the commanded soft limit
        // encoder.setPosition(Constants.TurretConstants.hoodSoftLimit1);
        homingInProgress = false;
        reset = true;
        SmartDashboard.putString("Hood/HomingStatus", reached ? "reached" : "timeout");
      }
    }
  }
}