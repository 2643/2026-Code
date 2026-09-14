// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

/**
 * Centralized logging for AdvantageScope. Everything relevant lives under a
 * single {@code AdvantageScope/} NetworkTables namespace so it's one click to
 * find in the AdvantageScope sidebar. All of these topics are also written to a
 * {@code .wpilog} file on the roboRIO (see {@link #start()}), so matches can be
 * replayed offline.
 *
 * <p>Poses are published as WPILib structs, which AdvantageScope renders
 * directly on its 2D/3D Field views; swerve module states feed its Swerve tab;
 * fuel positions are field "objects".
 */
public final class AdvantageScopeLogger {

  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("AdvantageScope");

  /* --- Field: robot position + camera/vision estimate + tracked fuel --- */
  private static final StructPublisher<Pose2d> robotPose =
      table.getStructTopic("Field/RobotPose", Pose2d.struct).publish();
  private static final StructPublisher<Pose2d> visionPose =
      table.getStructTopic("Field/VisionPose", Pose2d.struct).publish();
  private static final BooleanPublisher visionValid =
      table.getBooleanTopic("Field/VisionValid").publish();
  private static final DoublePublisher visionTagCount =
      table.getDoubleTopic("Field/VisionTagCount").publish();
  private static final StructArrayPublisher<Translation2d> fuelPositions =
      table.getStructArrayTopic("Field/Fuel", Translation2d.struct).publish();

  /* --- Swerve: chassis speeds + measured/target module states --- */
  private static final StructPublisher<ChassisSpeeds> chassisSpeeds =
      table.getStructTopic("Swerve/ChassisSpeeds", ChassisSpeeds.struct).publish();
  private static final StructArrayPublisher<SwerveModuleState> moduleStates =
      table.getStructArrayTopic("Swerve/ModuleStates", SwerveModuleState.struct).publish();
  private static final StructArrayPublisher<SwerveModuleState> moduleTargets =
      table.getStructArrayTopic("Swerve/ModuleTargets", SwerveModuleState.struct).publish();

  /* --- Subsystem states --- */
  private static final StringPublisher swivelState =
      table.getStringTopic("Swivel/State").publish();
  private static final StringPublisher swivelMode =
      table.getStringTopic("Swivel/Mode").publish();
  private static final DoublePublisher swivelPosition =
      table.getDoubleTopic("Swivel/Position").publish();
  private static final DoublePublisher swivelTarget =
      table.getDoubleTopic("Swivel/Target").publish();
  private static final BooleanPublisher swivelLimit =
      table.getBooleanTopic("Swivel/LimitSwitch").publish();

  private static final DoublePublisher hoodPosition =
      table.getDoubleTopic("Hood/Position").publish();
  private static final DoublePublisher hoodTarget =
      table.getDoubleTopic("Hood/Target").publish();
  private static final DoublePublisher hoodAngle =
      table.getDoubleTopic("Hood/Angle").publish();
  private static final BooleanPublisher hoodAtPosition =
      table.getBooleanTopic("Hood/AtPosition").publish();

  private static final BooleanPublisher intakeRunning =
      table.getBooleanTopic("Intake/Running").publish();
  private static final DoublePublisher intakeSpeed =
      table.getDoubleTopic("Intake/Speed").publish();

  private static final StringPublisher storagePhase =
      table.getStringTopic("Storage/Phase").publish();
  private static final StringPublisher storageWheel =
      table.getStringTopic("Storage/Wheel").publish();
  private static final StringPublisher storageIndexer =
      table.getStringTopic("Storage/Indexer").publish();
  private static final DoublePublisher storageWheelSpeed =
      table.getDoubleTopic("Storage/WheelSpeed").publish();
  private static final DoublePublisher storageTargetWheelSpeed =
      table.getDoubleTopic("Storage/TargetWheelSpeed").publish();
  private static final DoublePublisher storageIndexSpeed =
      table.getDoubleTopic("Storage/IndexSpeed").publish();

  private AdvantageScopeLogger() {}

  /**
   * Start on-robot data logging. This records every NetworkTables value
   * (SmartDashboard, the swerve DriveState topics, everything published here,
   * etc.) plus Driver Station / joystick data to a {@code .wpilog} file, which
   * AdvantageScope opens for offline replay. Call once from {@code robotInit}.
   */
  public static void start() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * Publish the latest robot pose, swerve states, and subsystem states. Call
   * once per loop from {@code robotPeriodic} (after the CommandScheduler runs).
   */
  public static void update() {
    Swerve drivetrain = RobotContainer.drivetrain;
    if (drivetrain != null) {
      var state = drivetrain.getState();
      if (state != null) {
        if (state.Pose != null) {
          robotPose.set(state.Pose);
        }
        if (state.Speeds != null) {
          chassisSpeeds.set(state.Speeds);
        }
        if (state.ModuleStates != null) {
          moduleStates.set(state.ModuleStates);
        }
        if (state.ModuleTargets != null) {
          moduleTargets.set(state.ModuleTargets);
        }
      }
    }

    // if (RobotContainer.m_FuelTracker != null) {
    //   Translation2d[] fuel = RobotContainer.m_FuelTracker.getFuelFieldPositions();
    //   fuelPositions.set(fuel);
    // }

    var swivel = RobotContainer.m_Swivel;
    if (swivel != null) {
      swivelState.set(String.valueOf(swivel.getState()));
      swivelMode.set(String.valueOf(swivel.getMode()));
      swivelPosition.set(swivel.getSwivelPos());
      swivelTarget.set(frc.robot.subsystems.Swivel.swivelTarget);
      swivelLimit.set(swivel.getSwivelLimit());
    }

    var hood = RobotContainer.m_Hood;
    if (hood != null) {
      hoodPosition.set(hood.getHoodPos());
      hoodTarget.set(frc.robot.subsystems.Hood.hoodTarget);
      hoodAngle.set(hood.angle);
      hoodAtPosition.set(hood.isAtPosition());
    }

    var intake = RobotContainer.m_Intake;
    if (intake != null) {
      intakeRunning.set(intake.getSpeed());
      intakeSpeed.set(intake.speed);
    }

    var storage = RobotContainer.m_Storage;
    if (storage != null) {
      storagePhase.set(String.valueOf(storage.getPhase()));
      storageWheel.set(String.valueOf(storage.getWheel()));
      storageIndexer.set(String.valueOf(storage.getIndexer()));
      storageWheelSpeed.set(storage.wheelSpeed);
      storageTargetWheelSpeed.set(storage.targetWheelSpeed);
      storageIndexSpeed.set(storage.indexSpeed);
    }
  }

  /**
   * Publish the latest camera/vision pose estimate (e.g. Limelight MegaTag).
   * Call from the vision-processing code whenever a fresh estimate is computed.
   *
   * @param pose     field-relative pose from the camera, or {@code null} if none
   * @param valid    whether this estimate passed the validity gates
   * @param tagCount number of AprilTags used for the estimate
   */
  public static void updateVision(Pose2d pose, boolean valid, double tagCount) {
    if (pose != null) {
      visionPose.set(pose);
    }
    visionValid.set(valid);
    visionTagCount.set(tagCount);
  }
}
