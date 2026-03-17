package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Swerve;

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
        if (arr == null || arr.length < 6) {
            return Optional.empty();
        }

        // Assumed layout: [x, y, z, rx, ry, rz]
        double x = arr[0];
        double y = arr[1];
        double yawDeg = arr[5];
        double yawRad = Math.toRadians(yawDeg);

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
                swerve.addVisionMeasurement(p, Timer.getFPGATimestamp());
                try {
                    SmartDashboard.putBoolean("Limelight/LastPushed", true);
                    SmartDashboard.putNumber("Limelight/LastPushTime", Timer.getFPGATimestamp());
                } catch (Throwable t) {
                }
            });
        } catch (Throwable t) {
            // ignore
        }
        m_notifier = new Notifier(() -> {
            try {
                getLatestPose().ifPresent(p -> {
                    swerve.addVisionMeasurement(p, Timer.getFPGATimestamp());
                    try {
                        SmartDashboard.putBoolean("Limelight/LastPushed", true);
                        SmartDashboard.putNumber("Limelight/LastPushTime", Timer.getFPGATimestamp());
                    } catch (Throwable t) {
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

    /**
     * Returns the most recently-read Limelight pose, if any (estimator pose should be read from the Swerve
     * subsystem instead).
     */
    public Optional<Pose2d> getLastPose() {
        return Optional.ofNullable(m_lastPose);
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
}
