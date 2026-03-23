package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Deprecated: direct LimelightHelpers MegaTag2 flow is used in RobotContainer.
 */
@Deprecated
public class Limelight3 {
    public Limelight3(String tableName) {
    }

    public Optional<Pose2d> getLastPose() {
        return Optional.empty();
    }

    public Optional<double[]> getLastXY() {
        return Optional.empty();
    }

    public void startUpdating(Object swerve, double periodSeconds) {
    }

    public void stopUpdating() {
    }
}
