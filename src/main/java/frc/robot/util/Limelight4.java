package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/**
 * Lightweight helper for Limelight 4 integration.
 *
 * - Reads pose arrays from the Limelight network table (botpose / botpose_wpiblue / botpose_wpired)
 *   and converts them to a {@link Pose2d} (x, y, yaw).
 * - Can periodically push vision poses into a {@link Swerve} instance using
 *   {@link Swerve#addVisionMeasurement(Pose2d, double)}.
 * - Exposes simple setters for pipeline/LED/camMode so the robot code can update Limelight settings at runtime.
 */
public class Limelight4 {
    private final NetworkTable m_table;
    private final NetworkTableEntry m_botpose;
    private final NetworkTableEntry m_botposeBlue;
    private final NetworkTableEntry m_botposeRed;
    private final NetworkTableEntry m_pipeline;
    private final NetworkTableEntry m_ledMode;
    private final NetworkTableEntry m_camMode;

    private Notifier m_notifier;
    /** If true, immediately reset the drivetrain pose to the vision pose when a tag is visible.
     * If false, vision measurements are fused via addVisionMeasurement (Kalman filter).
     */
    // If true, reset the drivetrain pose immediately when a valid Limelight pose is seen.
    // This makes the robot pose jump immediately to the vision pose and hold that pose
    // after the target disappears. If false, measurements are fused via addVisionMeasurement().
    private boolean m_resetOnDetection = true;

    private double m_lastPushTime = 0.0;

    /** Create a Limelight helper for the given network table name (usually "limelight"). */
    public Limelight4(String tableName) {
        m_table = NetworkTableInstance.getDefault().getTable(tableName);
        m_botpose = m_table.getEntry("botpose");
        m_botposeBlue = m_table.getEntry("botpose_wpiblue");
        m_botposeRed = m_table.getEntry("botpose_wpired");
        m_pipeline = m_table.getEntry("pipeline");
        m_ledMode = m_table.getEntry("ledMode");
        m_camMode = m_table.getEntry("camMode");
    }

    /**
     * Attempts to read the latest pose reported by the Limelight. Returns empty if no valid pose is available.
     *
     * Note: Limelight pose arrays vary by firmware/configuration. This method assumes that the
     * pose array contains at least [x, y, z, rx, ry, rz] where x/y are meters and rz is yaw in degrees.
     * If your Limelight is configured differently adjust this parsing accordingly.
     */
    public Optional<Pose2d> getLatestPose() {
        double[] arr = new double[0];
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
            arr = m_botposeBlue.getDoubleArray(new double[0]);
        } else if (alliance == Alliance.Red) {
            arr = m_botposeRed.getDoubleArray(new double[0]);
        }
        if (arr == null || arr.length < 6) {
            arr = m_botpose.getDoubleArray(new double[0]);
        }
        // read 'tv' early: only accept poses when a target is visible
        double tv = m_table.getEntry("tv").getDouble(0.0);
        // publish raw array and useful debug keys so we can inspect what Limelight reports
        try {
            SmartDashboard.putNumberArray("Limelight/RawBotpose", arr == null ? new double[0] : arr);
            SmartDashboard.putNumber("Limelight/tv", tv);
            // try reading tag id(s) if available
            double[] tids = m_table.getEntry("tid").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("Limelight/tid", tids == null ? new double[0] : tids);
            // publish some other common keys
            SmartDashboard.putNumber("Limelight/ta", m_table.getEntry("ta").getDouble(0.0));
            SmartDashboard.putNumber("Limelight/tx", m_table.getEntry("tx").getDouble(0.0));
            SmartDashboard.putNumber("Limelight/ty", m_table.getEntry("ty").getDouble(0.0));
            SmartDashboard.putNumber("Limelight/latency", m_table.getEntry("tl").getDouble(0.0));
            // publish all keys (short list) so we can inspect what's present
            Set<String> keys = m_table.getKeys();
            if (keys != null && !keys.isEmpty()) {
                StringBuilder sb = new StringBuilder();
                int i = 0;
                for (String k : keys) {
                    if (i++ > 0) sb.append(',');
                    sb.append(k);
                    if (i >= 30) break;
                }
                SmartDashboard.putString("Limelight/Keys", sb.toString());
            }
        } catch (Throwable t) {
            // ignore dashboard write errors
        }

        // if no target visible, don't treat any botpose as valid
        if (tv < 0.5) {
            SmartDashboard.putBoolean("Limelight/HasBotpose", false);
            return Optional.empty();
        }

        if (arr == null || arr.length < 6) {
            SmartDashboard.putBoolean("Limelight/HasBotpose", false);
            return Optional.empty();
        }

        SmartDashboard.putBoolean("Limelight/HasBotpose", true);
        // If the limelight reports a visible target, log it so we can see events in DS
        try {
            if (tv > 0.5) {
                DriverStation.reportWarning("Limelight: target visible, pushing pose", false);
            }
        } catch (Throwable t) {
        }

        // Assumed layout: [x, y, z, rx, ry, rz]
        double x = arr[0];
        double y = arr[1];
        double yawDeg = arr[5];
        double yawRad = Math.toRadians(yawDeg);

        // Heuristic: some Limelight configs publish in cm or mm. If the reported
        // coordinates are unreasonably large (e.g. > 50), assume they're in cm and
        // convert to meters. Publish the assumed unit for debugging.
        double maxAbs = Math.max(Math.abs(x), Math.abs(y));
        if (maxAbs > 1000.0) {
            // mm -> meters
            x /= 1000.0;
            y /= 1000.0;
            SmartDashboard.putString("Limelight/AssumedUnits", "mm");
        } else if (maxAbs > 50.0) {
            // cm -> meters
            x /= 100.0;
            y /= 100.0;
            SmartDashboard.putString("Limelight/AssumedUnits", "cm");
        } else {
            SmartDashboard.putString("Limelight/AssumedUnits", "m");
        }

        Pose2d pose = new Pose2d(new Translation2d(x, y), new Rotation2d(yawRad));
    // store the last-read pose for quick access
        m_lastPose = pose;
        // publish X/Y to SmartDashboard/Shuffleboard for visibility
        try {
            SmartDashboard.putNumber("Limelight/LastX", pose.getX());
            SmartDashboard.putNumber("Limelight/LastY", pose.getY());
        } catch (Throwable t) {
            // ignore dashboard errors
        }
        return Optional.of(pose);
    }

    /**
     * Start periodic updates that push Limelight poses into the provided Swerve estimator.
     *
     * This will call {@link Swerve#addVisionMeasurement(Pose2d, double)} with the current FPGA timestamp
     * whenever a valid pose is available.
     *
     * @param swerve The drivetrain instance to update
     * @param periodSeconds How often to poll the Limelight (e.g. 0.1 - 0.5s)
     */
    public void startUpdating(Swerve swerve, double periodSeconds) {
        stopUpdating();
        // Immediately read and push the latest pose (if any) so estimator has a first measurement
        try {
            getLatestPose().ifPresent(p -> {
                double latencyMs = m_table.getEntry("tl").getDouble(0.0);
                if (latencyMs == 0.0) {
                    latencyMs = m_table.getEntry("botpose_latency").getDouble(0.0);
                }
                double latencySeconds = latencyMs / 1000.0;
                double ts = Timer.getFPGATimestamp() - latencySeconds;
                // If turret angle supplier is present, skip the initial push when the
                // turret is rotated away from forward (to avoid bad initial pose).
                if (m_turretAngleSupplier != null) {
                    try {
                        double angDeg = Math.toDegrees(m_turretAngleSupplier.get());
                        SmartDashboard.putNumber("Limelight/TurretAngleDeg", angDeg);
                        if (Math.abs(angDeg) > m_turretAngleToleranceDeg) {
                            SmartDashboard.putBoolean("Limelight/SkippedPush_TurretMoved", true);
                            return;
                        }
                    } catch (Throwable t) {
                        // ignore supplier exceptions and continue
                    }
                }
                // apply configured field transform before pushing
                Pose2d used = applyFieldTransform(p);
                // sanity-check the transformed coords before pushing to avoid
                // wildly wrong units/axes causing the estimator to jump.
                double ux = used.getX();
                double uy = used.getY();
                if (Double.isFinite(ux) && Double.isFinite(uy) && Math.abs(ux) < 50.0 && Math.abs(uy) < 50.0) {
                    swerve.addVisionMeasurement(used, ts);
                } else {
                    SmartDashboard.putBoolean("Limelight/SkippedPush_OutOfBounds", true);
                    return;
                }
                try {
                    SmartDashboard.putBoolean("Limelight/LastPushed", true);
                    SmartDashboard.putNumber("Limelight/LastPushTime", ts);
                } catch (Throwable t) {
                }
            });
        } catch (Throwable t) {
            // ignore
        }
        m_notifier = new Notifier(() -> {
            try {
                getLatestPose().ifPresent(p -> {
                    double latencyMs = m_table.getEntry("tl").getDouble(0.0);
                    if (latencyMs == 0.0) {
                        latencyMs = m_table.getEntry("botpose_latency").getDouble(0.0);
                    }
                    double latencySeconds = latencyMs / 1000.0;
                    double ts = Timer.getFPGATimestamp() - latencySeconds;

                    double tv = m_table.getEntry("tv").getDouble(0.0);
                    // If configured, reset the drivetrain pose immediately when a valid
                    // detection occurs. For turreted cameras we preserve the current
                    // gyro/heading (we only reset X/Y) because the camera rotation may
                    // not match the robot heading.
                    // apply field transform first
                    Pose2d transformed = applyFieldTransform(p);
                    // If we have a turret angle supplier, skip using vision while the
                    // turret is significantly rotated to avoid conflicting poses.
                    if (m_turretAngleSupplier != null) {
                        try {
                            double angDeg = Math.toDegrees(m_turretAngleSupplier.get());
                            SmartDashboard.putNumber("Limelight/TurretAngleDeg", angDeg);
                            if (Math.abs(angDeg) > m_turretAngleToleranceDeg) {
                                SmartDashboard.putBoolean("Limelight/SkippedPush_TurretMoved", true);
                                return;
                            } else {
                                SmartDashboard.putBoolean("Limelight/SkippedPush_TurretMoved", false);
                            }
                        } catch (Throwable t) {
                            // ignore supplier errors
                        }
                    }

                    // sanity-check the transformed coords before using them
                    double txUsed = transformed.getX();
                    double tyUsed = transformed.getY();
                    if (!Double.isFinite(txUsed) || !Double.isFinite(tyUsed) || Math.abs(txUsed) > 50.0 || Math.abs(tyUsed) > 50.0) {
                        SmartDashboard.putBoolean("Limelight/SkippedPush_OutOfBounds", true);
                        return;
                    }

                    if (m_resetOnDetection && tv > 0.5) {
                        try {
                            // For turreted cameras: ignore the rotation reported by the
                            // Limelight. Use only X/Y from vision and keep the drivetrain's
                            // current heading (so gyro isn't overwritten).
                            var currentHeading = swerve.getState().Pose.getRotation();
                            var usedPose = new edu.wpi.first.math.geometry.Pose2d(transformed.getX(), transformed.getY(), currentHeading);
                            swerve.resetPose(usedPose);
                            double now = Timer.getFPGATimestamp();
                            m_lastPushTime = now;
                            SmartDashboard.putBoolean("Limelight/LastPushed", true);
                            SmartDashboard.putNumber("Limelight/LastPushTime", ts);
                            SmartDashboard.putNumber("Limelight/UsedLastX", usedPose.getX());
                            SmartDashboard.putNumber("Limelight/UsedLastY", usedPose.getY());
                            SmartDashboard.putString("Limelight/LastPushMethod", "reset-pos-only");
                        } catch (Throwable t) {
                            // ignore
                        }
                    } else {
                        // default: fuse the measurement into the estimator, but replace
                        // the vision rotation with the current drivetrain heading so the
                        // camera cannot change the robot heading.
                        try {
                            var currentHeading = swerve.getState().Pose.getRotation();
                            var usedPose = new edu.wpi.first.math.geometry.Pose2d(transformed.getX(), transformed.getY(), currentHeading);
                            swerve.addVisionMeasurement(usedPose, ts);
                            SmartDashboard.putString("Limelight/LastPushMethod", "fuse-pos-only");
                            SmartDashboard.putBoolean("Limelight/LastPushed", true);
                            SmartDashboard.putNumber("Limelight/LastPushTime", ts);
                            SmartDashboard.putNumber("Limelight/UsedLastX", usedPose.getX());
                            SmartDashboard.putNumber("Limelight/UsedLastY", usedPose.getY());
                            m_lastPushTime = Timer.getFPGATimestamp();
                        } catch (Throwable t) {
                            // ignore
                        }
                    }
                });
            } catch (Throwable t) {
                // swallow to keep notifier alive
            }
        });
        m_notifier.startPeriodic(periodSeconds);
    }
    
    /** The last pose read from the Limelight (may be null if none read yet). */
    private volatile Pose2d m_lastPose = null;
    // Field transform applied to all vision poses before using them for reset/fusion.
    // Use setFieldTransform(...) or selectFieldPreset(...) to configure.
    private double m_fieldOffsetX = 0.0; // meters
    private double m_fieldOffsetY = 0.0; // meters
    private double m_fieldRotationDeg = 0.0; // degrees
    // Optional turret angle supplier (radians). If set, we can gate vision updates
    // when the turret is far from a known forward position to avoid bad poses
    // from a rotating camera.
    private Supplier<Double> m_turretAngleSupplier = null;
    private double m_turretAngleToleranceDeg = 5.0; // degrees
    // Axis flip helpers: if the Limelight's reported X or Y sign is opposite
    // of the robot/estimator convention, flip the axis here.
    private boolean m_flipFieldX = false;
    private boolean m_flipFieldY = false;
    // If true, swap the reported X/Y axes from the Limelight before applying transforms.
    // Some camera configs report axes in a swapped order relative to the field coordinate convention.
    private boolean m_swapFieldXY = false;

    /**
     * Returns the most recently-read Limelight pose, if any (estimator pose should be read from the Swerve
     * subsystem instead).
     */
    public Optional<Pose2d> getLastPose() {
        return Optional.ofNullable(m_lastPose);
    }

    /** Apply the configured field transform to a pose (rotation then translation).
     * This lets you remap Limelight-reported field coordinates into a different
     * field origin/orientation (useful when switching between field revisions).
     */
    private Pose2d applyFieldTransform(Pose2d in) {
        if (in == null) return null;
        // rotate the input pose by m_fieldRotationDeg around origin, then translate
    // apply optional axis flips first (some Limelight configs report
    // X/Y with opposite signs from the robot convention).
    double sx = m_flipFieldX ? -1.0 : 1.0;
    double sy = m_flipFieldY ? -1.0 : 1.0;
        var srcTrans = new Translation2d(in.getX() * sx, in.getY() * sy);
        if (m_swapFieldXY) {
            srcTrans = new Translation2d(in.getY() * sy, in.getX() * sx);
        }
    var rot = new Rotation2d(Math.toRadians(m_fieldRotationDeg));
    var rotated = new Pose2d(
        srcTrans.rotateBy(rot),
        in.getRotation().plus(rot));
        var translated = new Pose2d(
                rotated.getX() + m_fieldOffsetX,
                rotated.getY() + m_fieldOffsetY,
                rotated.getRotation());
        return translated;
    }

    /** Set a manual field transform (offset in meters and rotation in degrees).
     * This transform is applied to all Limelight poses before they are used to
     * reset or fuse into the drivetrain estimator.
     */
    public void setFieldTransform(double offsetXMeters, double offsetYMeters, double rotationDegrees) {
        m_fieldOffsetX = offsetXMeters;
        m_fieldOffsetY = offsetYMeters;
        m_fieldRotationDeg = rotationDegrees;
        try {
            SmartDashboard.putNumber("Limelight/FieldOffsetX", m_fieldOffsetX);
            SmartDashboard.putNumber("Limelight/FieldOffsetY", m_fieldOffsetY);
            SmartDashboard.putNumber("Limelight/FieldRotationDeg", m_fieldRotationDeg);
            SmartDashboard.putBoolean("Limelight/FlipFieldX", m_flipFieldX);
            SmartDashboard.putBoolean("Limelight/FlipFieldY", m_flipFieldY);
        } catch (Throwable t) {
        }
    }

    /** Flip the sign of Limelight-reported X/Y before applying transforms. Use
     * this when the Limelight coordinate sign doesn't match the robot convention.
     */
    public void setFieldAxisFlip(boolean flipX, boolean flipY) {
        m_flipFieldX = flipX;
        m_flipFieldY = flipY;
        try {
            SmartDashboard.putBoolean("Limelight/FlipFieldX", m_flipFieldX);
            SmartDashboard.putBoolean("Limelight/FlipFieldY", m_flipFieldY);
            SmartDashboard.putBoolean("Limelight/SwapFieldXY", m_swapFieldXY);
        } catch (Throwable t) {}
    }

    /** Swap the Limelight X/Y axes before transforms. Use when camera reports axes swapped. */
    public void setFieldAxisSwap(boolean swapXY) {
        m_swapFieldXY = swapXY;
        try {
            SmartDashboard.putBoolean("Limelight/SwapFieldXY", m_swapFieldXY);
        } catch (Throwable t) {}
    }

    /** Convenience to select a known field preset. Currently supports:
     *  - "2026-rebuilt" — choose the 2026 rebuilt field transform
     *  - "2025-reefscape" — choose the 2025 reefscape field transform
     *
     * The actual numeric offsets are conservative defaults (zero). Update the
     * numbers below if you know the precise translation/rotation between fields.
     */
    public void selectFieldPreset(String preset) {
        if (preset == null) return;
        switch (preset.trim().toLowerCase()) {
            case "2026-rebuilt":
                // TODO: replace these with the actual known offset/rotation between the
                // 2026 rebuilt field and the 2025 reefscape origin. These defaults are
                // zero (no transform) so the code is safe until you provide precise values.
                setFieldTransform(0.0, 0.0, 0.0);
                break;
            case "2025-reefscape":
            default:
                setFieldTransform(0.0, 0.0, 0.0);
                break;
        }
        try {
            SmartDashboard.putString("Limelight/FieldPreset", preset);
        } catch (Throwable t) {
        }
    }

    /**
     * Returns the X/Y coordinates (meters) of the most recently-read Limelight pose as a double[2] = {x, y}.
     * Returns Optional.empty() if no pose has been read yet.
     */
    public Optional<double[]> getLastXY() {
        return getLastPose().map(p -> new double[] { p.getX(), p.getY() });
    }
    /** Stop periodic updates (if running). */
    public void stopUpdating() {
        if (m_notifier != null) {
            m_notifier.close();
            m_notifier = null;
        }
    }

    // --- Simple runtime setters so robot code can update Limelight settings ---
    public void setPipeline(int pipeline) {
        m_pipeline.setDouble(pipeline);
    }

    /**
     * LED modes: 0 = pipeline default, 1 = off, 2 = blink, 3 = on (Limelight convention)
     */
    public void setLedMode(int mode) {
        m_ledMode.setDouble(mode);
    }

    /**
     * Camera modes: 0 = vision processor, 1 = driver camera (Limelight convention)
     */
    public void setCamMode(int mode) {
        m_camMode.setDouble(mode);
    }

    /** Provide a supplier that returns the turret/swivel angle in radians.
     * When set, Limelight4 will skip pushing vision poses while the turret is
     * rotated more than the configured tolerance (see setTurretAngleToleranceDeg()).
     */
    public void setTurretAngleSupplier(Supplier<Double> supplier) {
        m_turretAngleSupplier = supplier;
        try {
            SmartDashboard.putBoolean("Limelight/HasTurretSupplier", supplier != null);
        } catch (Throwable t) {
        }
    }

    /** Set the acceptable turret angle tolerance (degrees). If the turret is
     * rotated more than this absolute angle, vision updates will be skipped.
     */
    public void setTurretAngleToleranceDeg(double deg) {
        m_turretAngleToleranceDeg = deg;
        try {
            SmartDashboard.putNumber("Limelight/TurretAngleToleranceDeg", m_turretAngleToleranceDeg);
        } catch (Throwable t) {}
    }
}
