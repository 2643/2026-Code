package frc.robot.util;

import java.util.ArrayDeque;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Field-relative robot velocity for shoot-on-the-move that survives pushing matches.
 *
 * When the drive motors sit near the slip current the wheels are traction-limited
 * (pushing or being pushed), so wheel-encoder velocity is wrong -- often even the wrong
 * direction. In that case we blend toward a velocity computed from raw MegaTag2 positions,
 * or toward zero if no tags are visible.
 */
public final class ShotVelocityEstimator {
  private static final double kWindowSec = 0.35;      // vision samples kept for the velocity fit
  private static final double kMinSpanSec = 0.15;     // need at least this much history to fit
  private static final double kMaxSampleAgeSec = 0.15; // newest sample must be this fresh
  private static final double kTrustRecoverPerSec = 4.0; // trust drops instantly, recovers over ~0.25s

  private record Sample(double t, double x, double y) {}

  private static final ArrayDeque<Sample> samples = new ArrayDeque<>();
  private static StatusSignal<Current>[] driveCurrents;
  private static double trust = 1.0;
  private static double lastTrustTime = Timer.getFPGATimestamp();

  private ShotVelocityEstimator() {}

  /** Feed a raw (unfused) MegaTag2 position. Call once per loop with the latest MT2 estimate. */
  public static void addVisionSample(Translation2d pos, double timestampSeconds) {
    Sample last = samples.peekLast();
    // LimelightHelpers stamps each read with "now - latency", so re-reading the same camera
    // frame gives a new timestamp with an identical pose. Skip those duplicates.
    if (last != null && last.x() == pos.getX() && last.y() == pos.getY()) {
      return;
    }
    samples.addLast(new Sample(timestampSeconds, pos.getX(), pos.getY()));
    while (!samples.isEmpty() && timestampSeconds - samples.peekFirst().t() > kWindowSec) {
      samples.removeFirst();
    }
  }

  /** Least-squares velocity over the vision window, or null if there isn't enough fresh data. */
  private static Translation2d getVisionVelocity() {
    double now = Timer.getFPGATimestamp();
    while (!samples.isEmpty() && now - samples.peekFirst().t() > kWindowSec) {
      samples.removeFirst();
    }
    if (samples.size() < 3
        || now - samples.peekLast().t() > kMaxSampleAgeSec
        || samples.peekLast().t() - samples.peekFirst().t() < kMinSpanSec) {
      return null;
    }
    double tMean = 0, xMean = 0, yMean = 0;
    for (Sample s : samples) {
      tMean += s.t(); xMean += s.x(); yMean += s.y();
    }
    int n = samples.size();
    tMean /= n; xMean /= n; yMean /= n;
    double stt = 0, stx = 0, sty = 0;
    for (Sample s : samples) {
      double dt = s.t() - tMean;
      stt += dt * dt;
      stx += dt * (s.x() - xMean);
      sty += dt * (s.y() - yMean);
    }
    return stt > 1e-9 ? new Translation2d(stx / stt, sty / stt) : null;
  }

  /** 1.0 = wheels have traction, 0.0 = drive motors are at the slip current. */
  private static double updateTrust() {
    if (driveCurrents == null) {
      @SuppressWarnings("unchecked")
      StatusSignal<Current>[] signals = new StatusSignal[4];
      for (int i = 0; i < 4; i++) {
        signals[i] = RobotContainer.drivetrain.getModule(i).getDriveMotor().getStatorCurrent(false);
      }
      BaseStatusSignal.setUpdateFrequencyForAll(50, signals);
      driveCurrents = signals;
    }
    BaseStatusSignal.refreshAll(driveCurrents);
    double maxCurrent = 0;
    for (var c : driveCurrents) {
      maxCurrent = Math.max(maxCurrent, Math.abs(c.getValueAsDouble()));
    }

    double slipAmps = Constants.OperatorConstants.kSlipCurrent.in(edu.wpi.first.units.Units.Amps);
    double fullTrustFrac = SmartDashboard.getNumber("SOTM/FullTrustLoad", 0.70);
    double zeroTrustFrac = SmartDashboard.getNumber("SOTM/ZeroTrustLoad", 0.95);
    double load = maxCurrent / slipAmps;
    double rawTrust = MathUtil.clamp((zeroTrustFrac - load) / (zeroTrustFrac - fullTrustFrac), 0, 1);

    double now = Timer.getFPGATimestamp();
    double dt = now - lastTrustTime;
    lastTrustTime = now;
    // Drop immediately when traction is lost; recover gradually so the wheels have
    // time to spin back down to the real speed before we trust them again.
    trust = rawTrust < trust ? rawTrust : Math.min(rawTrust, trust + kTrustRecoverPerSec * dt);

    SmartDashboard.putNumber("SOTM/DriveLoad", load);
    return trust;
  }

  /** Best field-relative velocity (m/s) of the robot center, given the wheel-odometry velocity. */
  public static Translation2d getFieldVelocity(double wheelVx, double wheelVy) {
    double t = updateTrust();
    Translation2d vision = getVisionVelocity();
    Translation2d fallback = vision != null ? vision : Translation2d.kZero;
    Translation2d wheel = new Translation2d(wheelVx, wheelVy);
    Translation2d out = wheel.times(t).plus(fallback.times(1 - t));

    SmartDashboard.putNumber("SOTM/Trust", t);
    SmartDashboard.putBoolean("SOTM/VisionVelValid", vision != null);
    SmartDashboard.putNumber("SOTM/WheelVelX", wheelVx);
    SmartDashboard.putNumber("SOTM/WheelVelY", wheelVy);
    SmartDashboard.putNumber("SOTM/VisionVelX", vision != null ? vision.getX() : 0);
    SmartDashboard.putNumber("SOTM/VisionVelY", vision != null ? vision.getY() : 0);
    return out;
  }
}
