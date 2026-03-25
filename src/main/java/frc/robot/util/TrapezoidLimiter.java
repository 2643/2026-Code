package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Small helper to apply independent slew-rate limiting to X, Y and rotational inputs.
 */
public class TrapezoidLimiter {
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter omegaLimiter;
    private double xRate;
    private double yRate;
    private double omegaRate;

    /**
     * Create a limiter.
     * @param xRate max change in m/s per second for X
     * @param yRate max change in m/s per second for Y
     * @param omegaRate max change in rad/s per second for rotation
     */
    public TrapezoidLimiter(double xRate, double yRate, double omegaRate) {
        this.xRate = xRate;
        this.yRate = yRate;
        this.omegaRate = omegaRate;
        this.xLimiter = new SlewRateLimiter(xRate);
        this.yLimiter = new SlewRateLimiter(yRate);
        this.omegaLimiter = new SlewRateLimiter(omegaRate);
    }

    /**
     * Reset internal state to the provided values (call when robot state jumps).
     */
    public void reset(double x, double y, double omega) {
        xLimiter.reset(x);
        yLimiter.reset(y);
        omegaLimiter.reset(omega);
    }

    public void setRates(double xRate, double yRate, double omegaRate) {
        if (this.xRate != xRate) {
            this.xRate = xRate;
            this.xLimiter = new SlewRateLimiter(xRate);
        }
        if (this.yRate != yRate) {
            this.yRate = yRate;
            this.yLimiter = new SlewRateLimiter(yRate);
        }
        if (this.omegaRate != omegaRate) {
            this.omegaRate = omegaRate;
            this.omegaLimiter = new SlewRateLimiter(omegaRate);
        }
    }

    public double[] getRates() {
        return new double[] { xRate, yRate, omegaRate };
    }

    /**
     * Calculate smoothed values. Returns array {x, y, omega}.
     */
    public double[] calculate(double x, double y, double omega) {
        double nx = xLimiter.calculate(x);
        double ny = yLimiter.calculate(y);
        double no = omegaLimiter.calculate(omega);
        return new double[] { nx, ny, no };
    }
}
