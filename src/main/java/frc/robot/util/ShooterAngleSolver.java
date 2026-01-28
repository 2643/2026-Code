// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Utility class to solve for the optimal shooter angle given a horizontal distance.
 * Uses projectile motion physics to achieve both position AND impact angle constraints.
 * 
 * PHYSICS:
 * Given: initial velocity v0, target position (x, y), desired impact angle φ
 * Find: launch angle θ
 * 
 * Equations:
 *   Time of flight: t = x / (v0 * cos(θ))
 *   Height reached: y = v0*sin(θ)*t - 0.5*g*t²
 *   Impact velocity: v_y = v0*sin(θ) - g*t, v_x = v0*cos(θ)
 *   Impact angle: tan(φ) = v_y / v_x
 * 
 * We solve this by trying different launch angles and checking both:
 * 1. Does it reach the right height?
 * 2. Does it have the right impact angle?
 */
public class ShooterAngleSolver {

    static final double g = 9.81; // gravity in m/s^2

    /**
     * Solves for the launch angle (theta) needed to hit a target with a specific impact angle.
     * 
     * Uses binary search to find the angle that satisfies both the position constraint
     * (reaching the target) and the velocity constraint (arriving at the desired angle).
     * 
     * @param v0  Initial velocity of the projectile (m/s)
     * @param dx  Horizontal distance to target (meters)
     * @param dy  Vertical distance to target (meters, positive = above launcher)
     * @param phi Desired impact angle (radians, negative = downward, e.g., -60° = -π/3)
     * @return Launch angle in radians
     */
    public static double solveTheta(
            double v0,
            double dx,
            double dy,
            double phi   // radians, negative for downward
    ) {
        // Binary search for the launch angle
        // We know it must be between 0 and 90 degrees
        double thetaLow = 0.0;
        double thetaHigh = Math.PI / 2.0 - 0.01; // Just under 90 degrees
        
        double targetTanPhi = Math.tan(phi);
        
        // Binary search iteration
        for (int iteration = 0; iteration < 100; iteration++) {
            double theta = (thetaLow + thetaHigh) / 2.0;
            
            // Calculate trajectory with this launch angle
            double cosTheta = Math.cos(theta);
            double sinTheta = Math.sin(theta);
            
            // Time to reach horizontal distance dx
            double t = dx / (v0 * cosTheta);
            
            // Height at that time
            double y = v0 * sinTheta * t - 0.5 * g * t * t;
            
            // Velocity at that time
            double vy = v0 * sinTheta - g * t;
            double vx = v0 * cosTheta;
            
            // Impact angle at that time
            double actualTanPhi = vy / vx;
            
            // Check if we're hitting too high or too low
            // For steeper angles (higher launch angle), the ball goes higher and arrives steeper
            // For shallower angles, the ball is lower and arrives more shallow
            
            // We want to match the impact angle primarily
            // If actual impact is steeper (more negative) than target, we launched too steep
            if (actualTanPhi < targetTanPhi) {
                // Impact is too steep, reduce launch angle
                thetaHigh = theta;
            } else {
                // Impact is too shallow, increase launch angle  
                thetaLow = theta;
            }
            
            // Check for convergence
            if (Math.abs(thetaHigh - thetaLow) < 0.0001) {
                break;
            }
        }
        
        return (thetaLow + thetaHigh) / 2.0;
    }

    /**
     * Converts radians to degrees
     */
    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Converts degrees to radians
     */
    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }
}
