// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelVisionConstants;

/**
 * Tracks fuel on the field using PhotonVision object-detection cameras.
 *
 * <p>Every loop, for each camera, this subsystem:
 * <ol>
 *   <li>Reads the latest frame from PhotonVision.</li>
 *   <li>For each detected fuel, uses the target's pitch to compute how far away it is
 *       (the camera's height/tilt + the fuel's known radius fix the geometry, since
 *       fuel sits on the floor).</li>
 *   <li>Uses the target's yaw to place it left/right, giving a camera-relative position.</li>
 *   <li>Transforms camera-relative -&gt; robot-relative -&gt; field coordinates using the
 *       robot's current pose from the swerve pose estimator.</li>
 * </ol>
 *
 * <p>Detections from all cameras are merged (fuel seen by both cameras within
 * {@link FuelVisionConstants#kMergeToleranceMeters} counts once) into a live snapshot
 * array of field positions, available via {@link #getFuelFieldPositions()}.
 */
public class FuelTracker extends SubsystemBase {

  /** One fuel-detection camera plus its mounting geometry and latest detections. */
  private static class FuelCamera {
    final PhotonCamera camera;
    final double heightMeters;
    final double pitchRadians;
    final Translation2d offset;      // camera position on robot, +X fwd, +Y left
    final Rotation2d yawOffset;      // which way the camera faces vs robot forward

    // Latest detections from this camera (robot-relative and field frame).
    List<Translation2d> robotRel = new ArrayList<>();
    List<Translation2d> field = new ArrayList<>();

    FuelCamera(String name, double heightMeters, double pitchRadians,
        Translation2d offset, Rotation2d yawOffset) {
      this.camera = new PhotonCamera(name);
      this.heightMeters = heightMeters;
      this.pitchRadians = pitchRadians;
      this.offset = offset;
      this.yawOffset = yawOffset;
    }

    /** Recompute this camera's detections if a new frame is available. */
    void update(Pose2d robotPose) {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (results.isEmpty()) {
        return; // no new frame; keep previous detections
      }
      PhotonPipelineResult latest = results.get(results.size() - 1);

      List<Translation2d> newRobotRel = new ArrayList<>();
      List<Translation2d> newField = new ArrayList<>();

      if (latest.hasTargets()) {
        for (PhotonTrackedTarget target : latest.getTargets()) {
          // Horizontal floor distance from camera to the fuel's center.
          double distance = PhotonUtils.calculateDistanceToTargetMeters(
              heightMeters,
              FuelVisionConstants.kFuelCenterHeightMeters,
              pitchRadians,
              Math.toRadians(target.getPitch()));

          // Reject geometrically impossible / unreliable detections.
          if (!Double.isFinite(distance)
              || distance < FuelVisionConstants.kMinDetectionRangeMeters
              || distance > FuelVisionConstants.kMaxDetectionRangeMeters) {
            continue;
          }

          // Camera-relative position. PhotonVision yaw is positive-RIGHT,
          // WPILib rotations are positive-LEFT (CCW), hence the negation.
          Translation2d camToFuel = PhotonUtils.estimateCameraToTargetTranslation(
              distance, Rotation2d.fromDegrees(-target.getYaw()));

          // Camera frame -> robot frame.
          Translation2d robotToFuel = offset.plus(camToFuel.rotateBy(yawOffset));

          // Robot frame -> field frame.
          Translation2d fieldPos = robotPose.getTranslation()
              .plus(robotToFuel.rotateBy(robotPose.getRotation()));

          newRobotRel.add(robotToFuel);
          newField.add(fieldPos);
        }
      }

      robotRel = newRobotRel;
      field = newField;
    }
  }

  private final FuelCamera[] cameras;
  private final Supplier<Pose2d> robotPoseSupplier;

  /** Field-frame (blue-origin) positions of all fuel currently visible, merged across cameras. */
  private Translation2d[] fuelFieldPositions = new Translation2d[0];
  /** Robot-relative positions (+X forward, +Y left) of all fuel currently visible. */
  private Translation2d[] fuelRobotRelative = new Translation2d[0];

  /**
   * @param robotPoseSupplier supplier of the robot's current field pose
   *                          (e.g. {@code () -> drivetrain.getState().Pose})
   */
  public FuelTracker(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.cameras = new FuelCamera[] {
        new FuelCamera(
            FuelVisionConstants.kCameraName,
            FuelVisionConstants.kCameraHeightMeters,
            FuelVisionConstants.kCameraPitchRadians,
            FuelVisionConstants.kCameraOffset,
            FuelVisionConstants.kCameraYawOffset),
        new FuelCamera(
            FuelVisionConstants.kCameraName2,
            FuelVisionConstants.kCameraHeightMeters2,
            FuelVisionConstants.kCameraPitchRadians2,
            FuelVisionConstants.kCameraOffset2,
            FuelVisionConstants.kCameraYawOffset2),
    };
  }

  /** Field positions (meters, blue-alliance origin) of every fuel currently in view. */
  public Translation2d[] getFuelFieldPositions() {
    return fuelFieldPositions.clone();
  }

  /** Robot-relative positions (+X forward, +Y left, meters) of every fuel currently in view. */
  public Translation2d[] getFuelRobotRelative() {
    return fuelRobotRelative.clone();
  }

  /** True if at least one fuel is currently visible. */
  public boolean hasFuel() {
    return fuelFieldPositions.length > 0;
  }

  /** The field position of the fuel closest to the robot, if any is visible. */
  public Optional<Translation2d> getClosestFuel() {
    Translation2d[] robotRel = fuelRobotRelative;
    Translation2d[] field = fuelFieldPositions;
    if (field.length == 0) {
      return Optional.empty();
    }
    int closest = 0;
    double best = robotRel[0].getNorm();
    for (int i = 1; i < robotRel.length; i++) {
      double d = robotRel[i].getNorm();
      if (d < best) {
        best = d;