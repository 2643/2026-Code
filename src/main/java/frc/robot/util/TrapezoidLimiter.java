package frc.robot.util;

/**
 * Simple trapezoidal rate limiter for linear and angular velocities.
 * Uses independent per-axis acceleration limits and a timestep measured
 * on each call. Intended to be fast-ramping (tunable via constructor).
 */
public class TrapezoidLimiter {
    private double prevX = 0.0;
    private double prevY = 0.0;
    private double prevOmega = 0.0;

    private final double maxAccelLinear; // m/s^2
    private final double maxAccelAngular; // rad/s^2

    private long lastTimeNs = System.nanoTime();

    /**
     * @param maxAccelLinear linear acceleration limit (m/s^2)
     * @param maxAccelAngular angular acceleration limit (rad/s^2)
     */
    public TrapezoidLimiter(double maxAccelLinear, double maxAccelAngular) {
        this.maxAccelLinear = Math.abs(maxAccelLinear);
        this.maxAccelAngular = Math.abs(maxAccelAngular);
    }

    /**
     * Compute smoothed velocities given desired setpoints.
     * @param desiredX desired linear velocity X (m/s)
     * @param desiredY desired linear velocity Y (m/s)
     * @param desiredOmega desired angular velocity (rad/s)
     * @return array {smoothedX, smoothedY, smoothedOmega}
     */
    public synchronized double[] calculate(double desiredX, double desiredY, double desiredOmega) {
        long now = System.nanoTime();
        double dt = Math.max(1e-6, (now - lastTimeNs) * 1e-9);
        lastTimeNs = now;

        prevX = limitAxis(prevX, desiredX, maxAccelLinear, dt);
        prevY = limitAxis(prevY, desiredY, maxAccelLinear, dt);
        prevOmega = limitAxis(prevOmega, desiredOmega, maxAccelAngular, dt);

        return new double[] { prevX, prevY, prevOmega };
    }

    private double limitAxis(double prev, double desired, double maxAccel, double dt) {
        double delta = desired - prev;
        double maxDelta = maxAccel * dt;
        if (Math.abs(delta) <= maxDelta) {
            return desired;
        }
        return prev + Math.copySign(maxDelta, delta);
    }

    public synchronized void reset(double x, double y, double omega) {
        prevX = x;
        prevY = y;
        prevOmega = omega;
        lastTimeNs = System.nanoTime();
    }
}
