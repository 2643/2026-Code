// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swivel.States;
import frc.robot.commands.Turret.ResetSwivel;
import frc.robot.commands.Turret.AutoAim;
import frc.robot.commands.Turret.ResetHood;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;


public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  public static boolean isRed;
  public double minVoltage = 67;
  private final boolean kUseLimelight = true;
  private static final String kLimelightName = "limelight-allen";
    private static final String kTurretLimelightName = "limelight-bhavik";

  private static boolean kForceApplyVisionForTest = true; // disable force mode; use fused vision instead
  private boolean m_seededFromVision = false;
  // Whether we've synced the gyro (pigeon) to the Limelight-reported heading
  private boolean m_gyroSeededFromVision = false;
  // Whether we've allowed the Limelight to set the robot heading from vision
  // (only allowed once). After this is true, future vision updates may update
  // translation but must preserve the drivetrain heading to avoid changing
  // controller direction mid-run.
  private boolean m_headingSeededFromVision = false;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
      SmartDashboard.putNumber("Swivel Gear Ratio", Constants.TurretConstants.swivelGearRatio);
     Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            SmartDashboard.putString("Alliance", "Red");
            isRed = true;
      } else if (ally.get() == Alliance.Blue) {
          SmartDashboard.putString("Alliance", "Blue");
          isRed = false;
        }
      }
      SmartDashboard.putString("Station Number", DriverStation.getLocation().toString());
      SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
      SmartDashboard.putString("Game Specific Message", DriverStation.getGameSpecificMessage());
      SmartDashboard.putString("Field/LayoutHint", "2026-Rebuilt");
      CommandScheduler.getInstance().schedule(new AutoAim());
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Anti Rotation Multiplier", Constants.TurretConstants.antiMultiplier);
    SmartDashboard.putNumber("Virtual X", Constants.TurretConstants.virtualX);
    SmartDashboard.putNumber("Virtual Y", Constants.TurretConstants.virtualY);
    SmartDashboard.putNumber("Shoot dx", Constants.TurretConstants.dx);
    SmartDashboard.putNumber("Shoot dy", Constants.TurretConstants.dy);
    SmartDashboard.putNumber("real dx", Constants.TurretConstants.realdx);
    SmartDashboard.putNumber("real dy", Constants.TurretConstants.realdy);

    Constants.TurretConstants.antiMultiplier = SmartDashboard.getNumber("antiMultiplier", 0.2);
    if (minVoltage > RobotController.getBatteryVoltage()){
            minVoltage = RobotController.getBatteryVoltage();
        }
    SmartDashboard.putNumber("Min Voltage", minVoltage);
        
    CommandScheduler.getInstance().run();

    if (kUseLimelight) {
  var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
  double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

  // Always tell the Limelight the current robot heading (gyro-derived) so
  // MegaTag2 localization can use an accurate yaw when we switch to it.
  LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0, 0, 0, 0, 0);
  // Use MegaTag1 exclusively for all pose estimates.
    frc.robot.util.LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
  var TurretllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kTurretLimelightName);
      double id = LimelightHelpers.getFiducialID(kLimelightName);
      Pose2d idk = null;
      if (llMeasurement != null) {
        // prefer the full pose (x,y,theta) reported by the Limelight; don't overwrite
        // the reported yaw with the drivetrain heading. Using the LL yaw is critical
        // for correct field-relative pose updates.
        idk = llMeasurement.pose;
      }
      boolean hasMeasurement = llMeasurement != null && llMeasurement.pose != null;
      boolean hasTags = hasMeasurement && llMeasurement.tagCount > 0;
      boolean tv = LimelightHelpers.getTV(kLimelightName);
      boolean turnRateOk = Math.abs(omegaRps) < 99999.0;
      boolean validForVision = tv && hasTags && turnRateOk;

      boolean hasMeasurementTurret = TurretllMeasurement != null && TurretllMeasurement.pose != null && id == 10;

      SmartDashboard.putBoolean("LL/HasMeasurement", hasMeasurement);
      SmartDashboard.putBoolean("LL/TV", tv);
      SmartDashboard.putBoolean("LL/HasTags", hasTags);
      SmartDashboard.putBoolean("LL/TurnRateOk", turnRateOk);
      SmartDashboard.putBoolean("LL/ValidForVision", validForVision);
      SmartDashboard.putNumber("LL/ID", id);
      kForceApplyVisionForTest = SmartDashboard.getBoolean("LL/resetWithPose", kForceApplyVisionForTest);


      if (hasMeasurement) {
        SmartDashboard.putNumber("LL/PoseX", llMeasurement.pose.getX());
        SmartDashboard.putNumber("LL/PoseY", llMeasurement.pose.getY());
        SmartDashboard.putNumber("LL/TagCount", llMeasurement.tagCount);
        SmartDashboard.putNumber("LL/AvgTagArea", llMeasurement.avgTagArea);
        SmartDashboard.putNumber("LL/Timestamp", llMeasurement.timestampSeconds);
      }

      if (hasMeasurementTurret) {
        SmartDashboard.putNumber("LLTurret/PoseX", TurretllMeasurement.pose.getX());
        SmartDashboard.putNumber("LLTurret/PoseY", TurretllMeasurement.pose.getY());
        SmartDashboard.putNumber("LLTurret/TagCount", TurretllMeasurement.tagCount);
        SmartDashboard.putNumber("LLTurret/AvgTagArea", TurretllMeasurement.avgTagArea);
        SmartDashboard.putNumber("LLTurret/Timestamp", TurretllMeasurement.timestampSeconds);
      }

      if (!m_seededFromVision && validForVision) {
        // Only seed/reset pose if the robot is nearly still. This avoids
        // teleporting/resets while driving. Thresholds are tunable on dashboard.
        double maxLinear = SmartDashboard.getNumber("LL/SeedMaxLinear_mps", 0.2); // m/s
        double maxAngular = SmartDashboard.getNumber("LL/SeedMaxAngular_rps", 0.5); // rad/s
        var speeds = RobotContainer.drivetrain.getState().Speeds;
        double lin = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double ang = Math.abs(speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("LL/SeedRobotLin", lin);
        SmartDashboard.putNumber("LL/SeedRobotAng", ang);
        if (lin <= maxLinear && ang <= maxAngular) {
          // For the initial seed, accept full pose (translation + heading).
          RobotContainer.drivetrain.resetPose(llMeasurement.pose);
          m_seededFromVision = true;
          m_headingSeededFromVision = true;
          SmartDashboard.putBoolean("LL/SeededPose", true);
          // Seed the gyro/pigeon to the camera heading the first time we accept vision.
          if (!m_gyroSeededFromVision) {
            RobotContainer.drivetrain.seedFieldCentric();
            m_gyroSeededFromVision = true;
            SmartDashboard.putBoolean("LL/GyroSeededFromVision", true);
          }
        } else {
          SmartDashboard.putBoolean("LL/SeededPose", false);
        }
      }

      // Always fuse vision when valid; if you want a testing-only immediate reset
      // use the SmartDashboard flag. But for active updates, feed everything
      // valid into the estimator.
      if (validForVision && llMeasurement != null) {
        var visionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5); // [m, m, rad]; tune as needed
        // If we've already allowed vision to set the heading once, preserve
        // the drivetrain heading for future vision updates so we don't change
        // controller direction mid-run. Always allow translation updates.
        var poseForFusion = llMeasurement.pose;
        if (m_headingSeededFromVision && RobotContainer.drivetrain != null && RobotContainer.drivetrain.getState() != null) {
          var currentPose = RobotContainer.drivetrain.getState().Pose;
          poseForFusion = new Pose2d(llMeasurement.pose.getTranslation(), currentPose.getRotation());
        }
        RobotContainer.drivetrain.addVisionMeasurement(
          poseForFusion,
          llMeasurement.timestampSeconds,
          visionStdDevs
        );
        // Optional: if user requested forced reset for testing, reset immediately
        // Optional: if user requested forced reset for testing, apply flicker
        // filter first to avoid teleporting on small camera jitters.
        if (kForceApplyVisionForTest && idk != null) {
            double posThresh = SmartDashboard.getNumber("LL/FlickerPosThreshold", 0.2); // meters
            double angThreshDeg = SmartDashboard.getNumber("LL/FlickerAngleThresholdDeg", 5.0); // degrees
            var currentPose = RobotContainer.drivetrain.getState().Pose;
            double dist = currentPose.getTranslation().getDistance(idk.getTranslation());
            double currentTheta = currentPose.getRotation().getRadians();
            double idkTheta = idk.getRotation().getRadians();
            double dtheta = Math.toDegrees(Math.atan2(Math.sin(idkTheta - currentTheta), Math.cos(idkTheta - currentTheta)));
            double angDiff = Math.abs(dtheta);
            SmartDashboard.putNumber("LL/FlickerDist", dist);
            SmartDashboard.putNumber("LL/FlickerAngleDiff", angDiff);

            // Persistence counter: require N consecutive frames above threshold before allowing reset
            int N = 3; // frames
            int consecutive = (int) SmartDashboard.getNumber("LL/FlickerConsecutive", 0);
            boolean frameExceeds = dist > posThresh || angDiff > angThreshDeg; // OR logic
            if (frameExceeds) {
              consecutive = Math.min(consecutive + 1, N);
            } else {
              consecutive = 0;
            }
            SmartDashboard.putNumber("LL/FlickerConsecutive", consecutive);
            boolean allowReset = consecutive >= N;
            SmartDashboard.putBoolean("LL/ResetAllowed", allowReset);
            if (allowReset) {
              RobotContainer.drivetrain.resetPose(idk);
              // clear counter after reset to avoid repeated immediate resets
              SmartDashboard.putNumber("LL/FlickerConsecutive", 0);
            }
          }
      }
    }
  }
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
    }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      // CommandScheduler.getInstance().schedule(new AutoAim());
    } else  {
      if(RobotContainer.m_Swivel.getState() == States.INITIALIZING) {
        CommandScheduler.getInstance().schedule(new ResetHood());
        CommandScheduler.getInstance().schedule(new ResetSwivel());
      }
    }
  }

  
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Constants.TurretConstants.swivelGearRatio = SmartDashboard.getNumber("Swivel Gear Ratio", Constants.TurretConstants.swivelGearRatio);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
