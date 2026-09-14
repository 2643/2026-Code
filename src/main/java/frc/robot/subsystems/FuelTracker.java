// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.Iterator;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.HttpCamera;
// import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.FuelVisionConstants;

// /**
//  * Tracks fuel on the field using PhotonVision object-detection cameras.
//  *
//  * <p>Every loop, for each camera, this subsystem:
//  * <ol>
//  *   <li>Reads the latest frame from PhotonVision.</li>
//  *   <li>For each detected fuel, uses the target's pitch to compute how far away it is
//  *       (the camera's height/tilt + the fuel's known radius fix the geometry, since
//  *       fuel sits on the floor).</li>
//  *   <li>Uses the target's yaw to place it left/right, giving a camera-relative position.</li>
//  *   <li>Transforms camera-relative -&gt; robot-relative -&gt; field coordinates using the
//  *       robot's current pose from the swerve pose estimator.</li>
//  * </ol>
//  *
//  * <p>Detections are merged across cameras and fed into a short-memory tracker:
//  * a fuel stays in the array for {@link FuelVisionConstants#kFuelMemorySeconds}
//  * after it was last seen, so it doesn't vanish the instant it leaves the camera
//  * frame (e.g. right as the robot drives up to intake it). Remembered fuel is
//  * stored in FIELD coordinates, so its robot-relative position stays correct
//  * as the robot keeps moving.
//  */
// public class FuelTracker extends SubsystemBase {

//   /** One fuel-detection camera plus its mounting geometry and latest detections. */
//   private static class FuelCamera {
//     final PhotonCamera camera;
//     final double heightMeters;
//     final double pitchRadians;
//     final Translation2d offset;      // camera position on robot, +X fwd, +Y left
//     final Rotation2d yawOffset;      // which way the camera faces vs robot forward

//     /** Latest field-frame detections from this camera. */
//     List<Translation2d> field = new ArrayList<>();

//     FuelCamera(String name, double heightMeters, double pitchRadians,
//         Translation2d offset, Rotation2d yawOffset) {
//       this.camera = new PhotonCamera(name);
//       this.heightMeters = heightMeters;
//       this.pitchRadians = pitchRadians;
//       this.offset = offset;
//       this.yawOffset = yawOffset;
//     }

//     /** Recompute this camera's detections if a new frame is available. */
//     void update(Pose2d robotPose) {
//       List<PhotonPipelineResult> results = camera.getAllUnreadResults();
//       if (results.isEmpty()) {
//         return; // no new frame; keep previous detections
//       }
//       PhotonPipelineResult latest = results.get(results.size() - 1);

//       List<Translation2d> newField = new ArrayList<>();

//       if (latest.hasTargets()) {
//         for (PhotonTrackedTarget target : latest.getTargets()) {
//           // Horizontal floor distance from camera to the fuel's center.
//           double distance = PhotonUtils.calculateDistanceToTargetMeters(
//               heightMeters,
//               FuelVisionConstants.kFuelCenterHeightMeters,
//               pitchRadians,
//               Math.toRadians(target.getPitch()));

//           // Reject geometrically impossible / unreliable detections.
//           if (!Double.isFinite(distance)
//               || distance < FuelVisionConstants.kMinDetectionRangeMeters
//               || distance > FuelVisionConstants.kMaxDetectionRangeMeters) {
//             continue;
//           }

//           // Camera-relative position. PhotonVision yaw is positive-RIGHT,
//           // WPILib rotations are positive-LEFT (CCW), hence the negation.
//           Translation2d camToFuel = PhotonUtils.estimateCameraToTargetTranslation(
//               distance, Rotation2d.fromDegrees(-target.getYaw()));

//           // Camera frame -> robot frame.
//           Translation2d robotToFuel = offset.plus(camToFuel.rotateBy(yawOffset));

//           // Robot frame -> field frame.
//           newField.add(robotPose.getTranslation()
//               .plus(robotToFuel.rotateBy(robotPose.getRotation())));
//         }
//       }

//       field = newField;
//     }
//   }

//   /** A fuel we've seen recently, remembered in field coordinates. */
//   private static class TrackedFuel {
//     Translation2d fieldPos;
//     double lastSeenSeconds;

//     TrackedFuel(Translation2d fieldPos, double lastSeenSeconds) {
//       this.fieldPos = fieldPos;
//       this.lastSeenSeconds = lastSeenSeconds;
//     }
//   }

//   private final FuelCamera[] cameras;
//   private final Supplier<Pose2d> robotPoseSupplier;
//   private final List<TrackedFuel> tracked = new ArrayList<>();

//   /** Field-frame (blue-origin) positions of all currently-tracked fuel. */
//   private Translation2d[] fuelFieldPositions = new Translation2d[0];
//   /** Robot-relative positions (+X forward, +Y left) of all currently-tracked fuel. */
//   private Translation2d[] fuelRobotRelative = new Translation2d[0];

//   /**
//    * @param robotPoseSupplier supplier of the robot's current field pose
//    *                          (e.g. {@code () -> drivetrain.getState().Pose})
//    */
//   public FuelTracker(Supplier<Pose2d> robotPoseSupplier) {
//     this.robotPoseSupplier = robotPoseSupplier;
//     this.cameras = new FuelCamera[] {
//         new FuelCamera(
//             FuelVisionConstants.kCameraName,
//             FuelVisionConstants.kCameraHeightMeters,
//             FuelVisionConstants.kCameraPitchRadians,
//             FuelVisionConstants.kCameraOffset,
//             FuelVisionConstants.kCameraYawOffset),
//         new FuelCamera(
//             FuelVisionConstants.kCameraName2,
//             FuelVisionConstants.kCameraHeightMeters2,
//             FuelVisionConstants.kCameraPitchRadians2,
//             FuelVisionConstants.kCameraOffset2,
//             FuelVisionConstants.kCameraYawOffset2),
//     };

//     // Publish both PhotonVision MJPEG streams to CameraServer so they show up
//     // in Elastic / Shuffleboard / SmartDashboard camera widgets.
//     startDashboardStream("Fuel Cam 0", FuelVisionConstants.kCameraStreamURL);
//     startDashboardStream("Fuel Cam 1", FuelVisionConstants.kCameraStreamURL2);
//   }

//   private static void startDashboardStream(String name, String url) {
//     try {
//       CameraServer.startAutomaticCapture(new HttpCamera(name, url, HttpCameraKind.kMJPGStreamer));
//     } catch (Exception e) {
//       System.err.println("[FuelTracker] Could not start dashboard stream " + name + ": " + e);
//     }
//   }

//   /** Field positions (meters, blue-alliance origin) of every tracked fuel. */
//   public Translation2d[] getFuelFieldPositions() {
//     return fuelFieldPositions.clone();
//   }

//   /** Robot-relative positions (+X forward, +Y left, meters) of every tracked fuel. */
//   public Translation2d[] getFuelRobotRelative() {
//     return fuelRobotRelative.clone();
//   }

//   /** True if at least one fuel is currently tracked. */
//   public boolean hasFuel() {
//     return fuelFieldPositions.length > 0;
//   }

//   /** The field position of the tracked fuel closest to the robot, if any. */
//   public Optional<Translation2d> getClosestFuel() {
//     int i = closestIndex();
//     return i < 0 ? Optional.empty() : Optional.of(fuelFieldPositions[i]);
//   }

//   /** The robot-relative position (+X fwd, +Y left) of the closest tracked fuel, if any. */
//   public Optional<Translation2d> getClosestFuelRobotRelative() {
//     int i = closestIndex();
//     return i < 0 ? Optional.empty() : Optional.of(fuelRobotRelative[i]);
//   }

//   private int closestIndex() {
//     Translation2d[] robotRel = fuelRobotRelative;
//     if (robotRel.length == 0) {
//       return -1;
//     }
//     int closest = 0;
//     double best = robotRel[0].getNorm();
//     for (int i = 1; i < robotRel.length; i++) {
//       double d = robotRel[i].getNorm();
//       if (d < best) {
//         best = d;
//         closest = i;
//       }
//     }
//     return closest;
//   }

//   @Override
//   public void periodic() {
//     Pose2d robotPose = robotPoseSupplier.get();
//     double now = Timer.getFPGATimestamp();

//     for (FuelCamera cam : cameras) {
//       cam.update(robotPose);
//     }

//     updateTracked(now);
//     rebuildArrays(robotPose);
//     publishTelemetry();
//   }

//   /**
//    * Feeds fresh detections into the memory tracker: detections matching an existing
//    * tracked fuel (within kMergeToleranceMeters) refresh its position and timestamp,
//    * new ones are added, and anything unseen for kFuelMemorySeconds is dropped.
//    */
//   private void updateTracked(double now) {
//     for (FuelCamera cam : cameras) {
//       for (Translation2d detection : cam.field) {
//         TrackedFuel match = null;
//         double bestDist = FuelVisionConstants.kMergeToleranceMeters;
//         for (TrackedFuel t : tracked) {
//           double d = detection.getDistance(t.fieldPos);
//           if (d < bestDist) {
//             bestDist = d;
//             match = t;
//           }
//         }
//         if (match != null) {
//           match.fieldPos = detection;
//           match.lastSeenSeconds = now;
//         } else {
//           tracked.add(new TrackedFuel(detection, now));
//         }
//       }
//     }

//     // Expire fuel we haven't seen recently.
//     Iterator<TrackedFuel> it = tracked.iterator();
//     while (it.hasNext()) {
//       if (now - it.next().lastSeenSeconds > FuelVisionConstants.kFuelMemorySeconds) {
//         it.remove();
//       }
//     }
//   }

//   /** Rebuilds the output arrays; robot-relative uses the CURRENT pose so memory stays accurate. */
//   private void rebuildArrays(Pose2d robotPose) {
//     Translation2d[] field = new Translation2d[tracked.size()];
//     Translation2d[] robotRel = new Translation2d[tracked.size()];
//     Rotation2d inverseHeading = robotPose.getRotation().unaryMinus();
//     for (int i = 0; i < tracked.size(); i++) {
//       field[i] = tracked.get(i).fieldPos;
//       robotRel[i] = field[i].minus(robotPose.getTranslation()).rotateBy(inverseHeading);
//     }
//     fuelFieldPositions = field;
//     fuelRobotRelative = robotRel;
//   }

//   private void publishTelemetry() {
//     for (int i = 0; i < cameras.length; i++) {
//       SmartDashboard.putBoolean("Fuel/Camera " + i + " Connected", cameras[i].camera.isConnected());
//       SmartDashboard.putNumber("Fuel/Camera " + i + " Count", cameras[i].field.size());
//     }
//     SmartDashboard.putNumber("Fuel/Count", fuelFieldPositions.length);

//     // Flattened [x1, y1, x2, y2, ...] field coordinates for dashboards/logging.
//     Translation2d[] field = fuelFieldPositions;
//     double[] xy = new double[field.length * 2];
//     for (int i = 0; i < field.length; i++) {
//       xy[2 * i] = field[i].getX();
//       xy[2 * i + 1] = field[i].getY();
//     }
//     SmartDashboard.putNumberArray("Fuel/Field Positions XY", xy);

//     Optional<Translation2d> closest = getClosestFuel();
//     if (closest.isPresent()) {
//       SmartDashboard.putNumber("Fuel/Closest X", closest.get().getX());
//       SmartDashboard.putNumber("Fuel/Closest Y", closest.get().getY());
//     }
//   }
// }
