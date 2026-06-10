package frc.robot.subsystems;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;

/**
 * HubTester subsystem - allows logging successful shots into a CSV file and
 * exposes simple SmartDashboard controls for single-person tuning.
 *
 * Usage:
 * - Open Shuffleboard/SmartDashboard and set "HubTester/LogNow" to true to
 *   record a shot. The subsystem will append a CSV line and flip the flag
 *   back to false. The last entry is published to "HubTester/LastEntry".
 */
public class HubTester extends SubsystemBase {
    private final Path logPath = Path.of("/home/lvuser/hub_lookup_logs.csv");

    public HubTester() {
        // Put dashboard controls
        SmartDashboard.putString("HubTester/LogFilePath", logPath.toString());
        SmartDashboard.putBoolean("HubTester/LogNow", false);
        SmartDashboard.putString("HubTester/LastEntry", "");

        // Ensure file exists and write header if empty
        try {
            if (Files.notExists(logPath)) {
                Files.createDirectories(logPath.getParent());
                Files.writeString(logPath, "timestamp,dist_m,shooter_rps,hood_deg,time_of_flight\n",
                        StandardCharsets.UTF_8, StandardOpenOption.CREATE);
            }
        } catch (IOException e) {
            SmartDashboard.putString("HubTester/Error", "Failed to create log file: " + e.toString());
        }
        SmartDashboard.putBoolean("HubTester/ImportNow", false);
        SmartDashboard.putBoolean("HubTester/DumpToJava", false);
        SmartDashboard.putBoolean("HubTester/CSVToDashboard", false);
        SmartDashboard.putString("HubTester/CSVDump", "");
    }

    @Override
    public void periodic() {
        // Check dashboard flag to log a shot
        try {
            boolean logNow = SmartDashboard.getBoolean("HubTester/LogNow", false);
            if (logNow) {
                logShot();
                // Flip back
                SmartDashboard.putBoolean("HubTester/LogNow", false);
            }
            boolean importNow = SmartDashboard.getBoolean("HubTester/ImportNow", false);
            if (importNow) {
                try {
                    frc.robot.util.TurretUtil.importHubTableFromCsv(logPath);
                    SmartDashboard.putString("HubTester/Status", "Imported");
                } catch (IOException e) {
                    SmartDashboard.putString("HubTester/Error", "Import failed: " + e.toString());
                }
                SmartDashboard.putBoolean("HubTester/ImportNow", false);
            }

            boolean dumpNow = SmartDashboard.getBoolean("HubTester/DumpToJava", false);
            if (dumpNow) {
                String javaDump = frc.robot.util.TurretUtil.dumpHubTableAsJava();
                SmartDashboard.putString("HubTester/JavaDump", javaDump);
                SmartDashboard.putBoolean("HubTester/DumpToJava", false);
            }

            boolean csvToDash = SmartDashboard.getBoolean("HubTester/CSVToDashboard", false);
            if (csvToDash) {
                try {
                    String all = Files.readString(logPath, StandardCharsets.UTF_8);
                    SmartDashboard.putString("HubTester/CSVDump", all);
                    SmartDashboard.putString("HubTester/Status", "CSV dumped to dashboard");
                } catch (IOException e) {
                    SmartDashboard.putString("HubTester/Error", "Failed to read CSV: " + e.toString());
                }
                SmartDashboard.putBoolean("HubTester/CSVToDashboard", false);
            }
        } catch (Throwable t) {
            // ignore dashboard issues
        }
    }

    /** Read current robot state and append a CSV line describing a successful shot. */
    public void logShot() {
        try {
            // Gather data: distance to hub from drivetrain pose, shooter RPS from Storage/Turret, hood angle
            Pose2d pose = RobotContainer.drivetrain.getState().Pose;
            double dist = frc.robot.util.TurretUtil.getDistance(pose, frc.robot.util.TurretUtil.TargetType.HUB);

            // Shooter speed and time-of-flight can be approximated from lookup table
            double shooterRps = 0.0;
            double hoodDeg = 0.0;
            double tof = 0.0;
            try {
                var params = new frc.robot.util.HubLookUpTable().getParameters(dist);
                shooterRps = params.shooterSpeed;
                hoodDeg = params.trajectoryAngle;
                tof = params.timeOfFlight;
            } catch (Throwable t) {
                // fallback values
            }

            String line = String.format("%s,%.3f,%.3f,%.3f,%.3f\n", Instant.now().toString(), dist, shooterRps, hoodDeg, tof);
            Files.writeString(logPath, line, StandardCharsets.UTF_8, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            SmartDashboard.putString("HubTester/LastEntry", line.trim());
            SmartDashboard.putString("HubTester/Status", "Logged");
        } catch (IOException e) {
            SmartDashboard.putString("HubTester/Error", "IO error: " + e.toString());
        } catch (Throwable t) {
            SmartDashboard.putString("HubTester/Error", "Unexpected: " + t.toString());
        }
    }
}
