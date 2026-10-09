// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Turret.AutoAim;
import frc.robot.commands.Turret.ResetHood;
import frc.robot.commands.Turret.ResetSwivel;
import frc.robot.subsystems.Swivel.States;
import frc.robot.util.LimelightHelpers;


public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  public static boolean isRed;
  private final boolean kUseLimelight = true;
  private static final String limeAllen = "limelight-allen";
  // turret limelight unused; kept here for reference
  private static final String limeBhavik = "limelight-bhavik";

  private static boolean kForceApplyVisionForTest = true; // disable force mode; use fused vision instead
  private boolean m_seededFromVision = false;
  // Whether we've synced the gyro (pigeon) to the Limelight-reported heading
  private boolean m_gyroSeededFromVision = false;
  // Whether we've allowed the Limelight to set the robot heading from vision
  // (only allowed once). After this is true, future vision updates may update
  // translation but must preserve the drivetrain heading to avoid changing
  // controller direction mid-run.
  private boolean m_headingSeededFromVision = false;
  // Set by zero-gyro: seedFieldCentric() sets heading to the alliance-forward direction,
  // which is 180 off if the DS alliance doesn't match the side we're on. Re-seeding from
  // MegaTag1 restores the true field heading before MegaTag2 is used again.
  private static boolean s_visionReseedRequested = false;

  private final RobotContainer m_robotContainer;

  public static void requestVisionReseed() {
    s_visionReseedRequested = true;
  }

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  public static boolean isRed(){
    return isRed;
  }

  @Override
  public void robotInit() {
      SmartDashboard.putNumber("Swivel Gear Ratio", Constants.TurretConstants.swivelGearRatio);
     Optional<Alliance> ally = DriverStation.getAlliance();
      // if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            SmartDashboard.putString("Alliance", "Red");
            isRed = true;
      } else if (ally.get() == Alliance.Blue) {
          SmartDashboard.putString("Alliance", "Blue");
          isRed = false;
        }
      // }
      SmartDashboard.putString("Station Number", DriverStation.getLocation().toString());
      SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
      SmartDashboard.putString("Game Specific Message", DriverStation.getGameSpecificMessage());
      SmartDashboard.putString("Field/LayoutHint", "2026-Rebuilt");
      CommandScheduler.getInstance().schedule(new AutoAim());
      SignalLogger.setPath("/home/lvuser/logs/");
      SignalLogger.start();
      // Record all NetworkTables + Driver Station data to a .wpilog for AdvantageScope replay.
      frc.robot.util.AdvantageScopeLogger.start();
  }
  @Override

  public void disabledInit(){
  // SignalLogger.stop();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Virtual X", Constants.TurretConstants.virtualX);
    SmartDashboard.putNumber("Virtual Y", Constants.TurretConstants.virtualY);
    SmartDashboard.putNumber("Shoot dx", Constants.TurretConstants.dx);
    SmartDashboard.putNumber("Shoot dy", Constants.TurretConstants.dy);
    SmartDashboard.putNumber("real dx", Constants.TurretConstants.realdx);
    SmartDashboard.putNumber("real dy", Constants.TurretConstants.realdy);
    SmartDashboard.putBoolean("Alliance", isRed);

    // Read and write the same key so edits on the dashboard actually stick.
    Constants.TurretConstants.antiMultiplier = SmartDashboard.getNumber("Anti Rotation Multiplier", 0.2);
    SmartDashboard.putNumber("Anti Rotation Multiplier", Constants.TurretConstants.antiMultiplier);
    
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        
    CommandScheduler.getInstance().run();

    if (kUseLimelight) {
      // processLimelight(limeBhavik);
      processLimelight(limeAllen);
    }

    // Publish robot pose, swerve states, and subsystem states for AdvantageScope.
    frc.robot.util.AdvantageScopeLogger.update();
  }

  /**
   * Consolidated limelight/vision processing moved out of robotPeriodic to
   * improve readability. Behavior is identical to the previous inline code.
   */
  private void processLimelight(String lname) {
    String kLimelightName = lname;
    var driveState = RobotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    // Always tell the Limelight the current robot heading (gyro-derived) so
    // MegaTag2 localization can use an accurate yaw when we switch to it.
    LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0, 0, 0, 0, 0);

    // Use MegaTag1 exclusively for all pose estimates.
  frc.robot.util.LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
  // turret-specific limelight is unused now (we use the robot pose directly).
  // var turretLlMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kTurretLimelightName);
    double id = LimelightHelpers.getFiducialID(kLimelightName);
    Pose2d idPose = null;
    if (llMeasurement != null) {
      // prefer the full pose (x,y,theta) reported by the Limelight; don't overwrite
      // the reported yaw with the drivetrain heading. Using the LL yaw is critical
      // for correct field-relative pose updates.
      idPose = llMeasurement.pose;
    }

    boolean hasMeasurement = llMeasurement != null && llMeasurement.pose != null;
    boolean hasTags = hasMeasurement && llMeasurement.tagCount > 0;
    boolean tv = LimelightHelpers.getTV(kLimelightName);
    boolean turnRateOk = Math.abs(omegaRps) < 99999.0;
    boolean validForVision = tv && hasTags && turnRateOk;

    if(SmartDashboard.getString("Alliance", "Red").equals("Red"))
      isRed = true;
    else
      isRed = false;

  // turret-specific measurement not used anymore; commenting out
  // boolean hasMeasurementTurret = turretLlMeasurement != null && turretLlMeasurement.pose != null && id == 10;

    SmartDashboard.putBoolean("LL/HasMeasurement", hasMeasurement);
    SmartDashboard.putBoolean("LL/TV", tv);
    SmartDashboard.putBoolean("LL/HasTags", hasTags);
    SmartDashboard.putBoolean("LL/TurnRateOk", turnRateOk);
    SmartDashboard.putBoolean("LL/ValidForVision", validForVision);
    SmartDashboard.putNumber("LL/ID", id);
    kForceApplyVisionForTest = SmartDashboard.getBoolean("LL/resetWithPose", kForceApplyVisionForTest);

    // Log the camera pose estimate for AdvantageScope's field view.
    frc.robot.util.AdvantageScopeLogger.updateVision(
        hasMeasurement ? llMeasurement.pose : null,
        validForVision,
        hasMeasurement ? llMeasurement.tagCount : 0);

    if (hasMeasurement) {
      SmartDashboard.putNumber("LL/PoseX", llMeasurement.pose.getX());
      SmartDashboard.putNumber("LL/PoseY", llMeasurement.pose.getY());
      SmartDashboard.putNumber("LL/TagCount", llMeasurement.tagCount);
      SmartDashboard.putNumber("LL/AvgTagArea", llMeasurement.avgTagArea);
      SmartDashboard.putNumber("LL/Timestamp", llMeasurement.timestampSeconds);
    }

    // turret-specific telemetry disabled because turret limelight is not used
    // if (hasMeasurementTurret) {
    //   SmartDashboard.putNumber("LLTurret/PoseX", turretLlMeasurement.pose.getX());
    //   SmartDashboard.putNumber("LLTurret/PoseY", turretLlMeasurement.pose.getY());
    //   SmartDashboard.putNumber("LLTurret/TagCount", turretLlMeasurement.tagCount);
    //   SmartDashboard.putNumber("LLTurret/AvgTagArea", turretLlMeasurement.avgTagArea);
    //   SmartDashboard.putNumber("LLTurret/Timestamp", turretLlMeasurement.timestampSeconds);
    // }

    if (s_visionReseedRequested) {
      m_seededFromVision = false;
      s_visionReseedRequested = false;
    }

    // Seed/reset pose only when robot is nearly still to avoid teleporting while driving.
    if (!m_seededFromVision && validForVision) {
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
        // seedFieldCentric() resets the pose heading to the operator-forward direction,
        // which would throw away the vision heading we just set (and break MegaTag2).
        // Operator perspective is already handled in Swerve.periodic().
        // if (!m_gyroSeededFromVision) {
        //   RobotContainer.drivetrain.seedFieldCentric();
        //   m_gyroSeededFromVision = true;
        //   SmartDashboard.putBoolean("LL/GyroSeededFromVision", true);
        // }
      } else {
        SmartDashboard.putBoolean("LL/SeededPose", false);
      }
    }

    // Continuous fusion: MegaTag2 into the pose estimator (Kalman filter).
    // MT2 uses the gyro heading we send via SetRobotOrientation, so it is only valid
    // after the heading has been seeded above. Heading std dev is huge so vision never
    // fights the gyro on rotation; only X/Y get corrected.
    if (m_seededFromVision) {
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
      double omegaDegPerSec = Math.abs(Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond));
      boolean mt2Valid = mt2 != null && mt2.pose != null && mt2.tagCount > 0 && omegaDegPerSec < 720.0;
      SmartDashboard.putBoolean("LL/MT2Valid", mt2Valid);
      if (mt2Valid) {
        // Trust vision less the farther away the tags are; trust more with multiple tags.
        double xyStdDev = SmartDashboard.getNumber("LL/MT2BaseStdDev", 0.5)
            * (1.0 + mt2.avgTagDist * mt2.avgTagDist / 10.0)
            / (mt2.tagCount > 1 ? 2.0 : 1.0);
        SmartDashboard.putNumber("LL/MT2StdDev", xyStdDev);
        SmartDashboard.putNumber("LL/MT2PoseX", mt2.pose.getX());
        SmartDashboard.putNumber("LL/MT2PoseY", mt2.pose.getY());
        frc.robot.util.ShotVelocityEstimator.addVisionSample(mt2.pose.getTranslation(), mt2.timestampSeconds);
        RobotContainer.drivetrain.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
        );
      }
    }

    // OLD: constantly resetPose() to the Limelight pose. Replaced by addVisionMeasurement above.
    /*
    if (validForVision && llMeasurement != null) {
      var poseForFusion = llMeasurement.pose;
      if (m_headingSeededFromVision && RobotContainer.drivetrain != null && RobotContainer.drivetrain.getState() != null) {
        var currentPose = RobotContainer.drivetrain.getState().Pose;
        poseForFusion = new Pose2d(llMeasurement.pose.getTranslation(), currentPose.getRotation());
      }

      // Testing-only: optional immediate reset with flicker filter to avoid teleporting on small jitters.
      if (kForceApplyVisionForTest && idPose != null) {
        double posThresh = SmartDashboard.getNumber("LL/FlickerPosThreshold", 0.3); // meters
        double posThreshmax = SmartDashboard.getNumber("LL/FlickerPosThresholdMax", 1.2); // meters

        double angThreshDeg = SmartDashboard.getNumber("LL/FlickerAngleThresholdDeg", 10.0); // degrees
        double angThreshDegMax = SmartDashboard.getNumber("LL/FlickerAngleThresholdDegMax", 50.0); // degrees

        var currentPose = RobotContainer.drivetrain.getState().Pose;
        double dist = currentPose.getTranslation().getDistance(idPose.getTranslation());
        double currentTheta = currentPose.getRotation().getRadians();
        double idkTheta = idPose.getRotation().getRadians();
        double dtheta = Math.toDegrees(Math.atan2(Math.sin(idkTheta - currentTheta), Math.cos(idkTheta - currentTheta)));
        double angDiff = Math.abs(dtheta);
        SmartDashboard.putNumber("LL/FlickerDist", dist);
        SmartDashboard.putNumber("LL/FlickerAngleDiff", angDiff);

        // Persistence counter: require N consecutive frames above threshold before allowing reset
        int N = 10; // frames
        int consecutive = (int) SmartDashboard.getNumber("LL/FlickerConsecutive", 0);
        boolean frameExceeds = dist > posThresh || angDiff > angThreshDeg; // OR logic
        boolean frameTooFar = dist > posThreshmax || angDiff > angThreshDegMax;
        if (frameTooFar) {
          consecutive = Math.min(consecutive + 1, N);
        } else {
          consecutive = 0;
        }

        SmartDashboard.putNumber("LL/FlickerConsecutive", consecutive);
        boolean allowReset = consecutive >= N;
        SmartDashboard.putBoolean("LL/ResetAllowed", allowReset);
        if (allowReset || (frameExceeds && !frameTooFar)) {
          RobotContainer.drivetrain.resetPose(poseForFusion);
          // clear counter after reset to avoid repeated immediate resets
          SmartDashboard.putNumber("LL/FlickerConsecutive", 0);
        }
      }
    }
    */
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
