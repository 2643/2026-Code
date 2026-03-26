package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Swerve;

//TO DELETE GANG

/**
 * Simple compatibility wrapper named Limelight3 that delegates to the existing Limelight4
 * implementation. This lets code reference Limelight3 (e.g. from limelight-bhavik) while
 * reusing the proven Limelight4 logic in this repo.
 */
public class Limelight3 {
    private final Limelight3 delegate;

    public Limelight3(String tableName) {
        this.delegate = new Limelight3(tableName);
    }

    public Optional<Pose2d> getLastPose() {
        return delegate.getLastPose();
    }

    public Optional<double[]> getLastXY() {
        return delegate.getLastXY();
    }

    public void selectFieldPreset(String preset) {
        delegate.selectFieldPreset(preset);
    }

    public void setFieldAxisFlip(boolean flipX, boolean flipY) {
        delegate.setFieldAxisFlip(flipX, flipY);
    }

    public void startUpdating(Swerve swerve, double periodSeconds) {
        delegate.startUpdating(swerve, periodSeconds);
    }

    public void stopUpdating() {
        delegate.stopUpdating();
    }

    // Pass-through setters for common runtime configurations
    public void setPipeline(int pipeline) { delegate.setPipeline(pipeline); }
    public void setLedMode(int mode) { delegate.setLedMode(mode); }
    public void setCamMode(int mode) { delegate.setCamMode(mode); }
    public void setTurretAngleSupplier(java.util.function.Supplier<Double> s) { delegate.setTurretAngleSupplier(s); }
    public void setTurretAngleToleranceDeg(double d) { delegate.setTurretAngleToleranceDeg(d); }
}
