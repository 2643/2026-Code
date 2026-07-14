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
 * Tracks fuel on the field using a PhotonVision object-detection camera.
 *
 * <p>Every loop, this subsystem:
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
 * <p>The result is a live snapshot array of field positions of every fuel the camera
 * currently sees, available via {@link #getFuelFieldPositions()}.
 */
public class FuelTracker extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera(FuelVisionConstants.kCameraName);
  private final Supplier<Pose2d> robotPoseSupplier;

  /** Field-frame (blue-origin) positions of all fuel currently visible. */
  private Translation2d[] fuelFieldPositions = new Translation2d[0];
  /** Robot-relative positions (+X forward, +Y left) of all fuel currently visible. */
  private Translation2d[] fuelRobotRelative = new Translation2d[0];

  private boolean cameraConnected = false;

  /**
   * @param robotPoseSupplier supplier of the robot's current field pose
   *                          (e.g. {@code () -> drivetrain.getState().Pose})
   */
  public FuelTracker(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
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
        closest = i;
      }
    }
    return Optional.of(field[closest]);
  }

  @Override
  public void periodic() {
    cameraConnected = camera.isConnected();

    // getAllUnreadResults() returns every frame since our last call.
    // We only care about the newest one for a live snapshot.
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      PhotonPipelineResult latest = results.get(results.size() - 1);
      updateFromResult(latest);
    }

    publishTelemetry();
  }

  private void updateFromResult(PhotonPipelineResult result) {
    ArrayList<Translation2d> robotRel = new ArrayList<>();
    ArrayList<Translation2d> field = new ArrayList<>();

    // NOTE: for better accuracy under fast driving, sample the pose at
    // result.getTimestampSeconds() instead of using the current pose.
    Pose2d robotPose = robotPoseSupplier.get();

    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        // Horizontal floor distance from camera to the fuel's center.
        // Works because the fuel sits on the floor at a known height (its radius).
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            FuelVisionConstants.kCameraHeightMeters,
            FuelVisionConstants.kFuelCenterHeightMeters,
            FuelVisionConstants.kCameraPitchRadians,
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

        // Camera frame -> robot frame (account for where the camera sits & faces).
        Translation2d robotToFuel = FuelVisionConstants.kCameraOffset
            .plus(camToFuel.rotateBy(FuelVisionConstants.kCameraYawOffset));

        // Robot frame -> field frame using the robot's pose.
        Translation2d fieldPos = robotPose.getTranslation()
            .plus(robotToFuel.rotateBy(robotPose.getRotation()));

        robotRel.add(robotToFuel);
        field.add(fieldPos);
      }
    }

    fuelRobotRelative = robotRel.toArray(new Translation2d[0]);
    fuelFieldPositions = field.toArray(new Translation2d[0]);
  }

  private void publishTelemetry() {
    SmartDashboard.putBoolean("Fuel/Camera Connected", cameraConnected);
    SmartDashboard.putNumber("Fuel/Count", fuelFieldPositions.length);

    // Flattened [x1, y1, x2, y2, ...] field coordinates for dashboards/logging.
    Translation2d[] field = fuelFieldPositions;
    double[] xy = new double[field.length * 2];
    for (int i = 0; i < field.length; i++) {
      xy[2 * i] = field[i].getX();
      xy[2 * i + 1] = field[i].getY();
    }
    SmartDashboard.putNumberArray("Fuel/Field Positions XY", xy);

    Optional<Translation2d> closest = getClosestFuel();
    if (closest.isPresent()) {
      SmartDashboard.putNumber("Fuel/Closest X", closest.get().getX());
      SmartDashboard.putNumber("Fuel/Closest Y", closest.get().getY());
    }
  }
}
