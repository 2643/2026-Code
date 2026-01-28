// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ShooterConstants;

/**
 * Standalone test to verify the shooter angle formula produces correct results.
 * Run this to check if the physics calculations are working.
 */
public class FormulaVerifier {
    
    /**
     * Test the formula with various distances and print the results.
     * This helps verify the math is correct.
     */
    public static void runVerification() {
        System.out.println("========================================");
        System.out.println("SHOOTER ANGLE FORMULA VERIFICATION");
        System.out.println("========================================");
        System.out.println();
        
        System.out.println("Constants being used:");
        System.out.println("  Ball Velocity: " + ShooterConstants.BALL_VELOCITY_MPS + " m/s");
        System.out.println("  Height Delta: " + ShooterConstants.TARGET_HEIGHT_DELTA_M + " m");
        System.out.println("  Impact Angle: " + Math.toDegrees(ShooterConstants.IMPACT_ANGLE_RAD) + "°");
        System.out.println("  Hood Range: " + ShooterConstants.MIN_HOOD_ANGLE_DEG + "° - " + ShooterConstants.MAX_HOOD_ANGLE_DEG + "°");
        System.out.println();
        
        System.out.println("Testing at various distances:");
        System.out.println("Distance (m) | Launch Angle (°) | In Range?");
        System.out.println("-------------|------------------|----------");
        
        double[] testDistances = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
        
        for (double distance : testDistances) {
            double angleRad = ShooterAngleSolver.solveTheta(
                ShooterConstants.BALL_VELOCITY_MPS,
                distance,
                ShooterConstants.TARGET_HEIGHT_DELTA_M,
                ShooterConstants.IMPACT_ANGLE_RAD
            );
            
            double angleDeg = Math.toDegrees(angleRad);
            boolean inRange = angleDeg >= ShooterConstants.MIN_HOOD_ANGLE_DEG && 
                            angleDeg <= ShooterConstants.MAX_HOOD_ANGLE_DEG;
            
            String rangeIndicator = inRange ? "✓ YES" : "✗ NO";
            System.out.printf("   %4.1f      |     %5.1f°       | %s%n", 
                            distance, angleDeg, rangeIndicator);
        }
        
        System.out.println();
        System.out.println("========================================");
        
        // Now verify the physics by simulating a shot
        System.out.println("PHYSICS VERIFICATION (2.0m shot):");
        System.out.println("========================================");
        
        double testDist = 2.0;
        double launchAngleRad = ShooterAngleSolver.solveTheta(
            ShooterConstants.BALL_VELOCITY_MPS,
            testDist,
            ShooterConstants.TARGET_HEIGHT_DELTA_M,
            ShooterConstants.IMPACT_ANGLE_RAD
        );
        
        // Simulate the trajectory
        double v0 = ShooterConstants.BALL_VELOCITY_MPS;
        double theta = launchAngleRad;
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);
        
        // Time to reach target distance
        double t = testDist / vx;
        
        // Calculate final position
        double finalY = vy * t - 0.5 * 9.81 * t * t;
        
        // Calculate final velocity components
        double finalVy = vy - 9.81 * t;
        double finalVx = vx; // horizontal velocity stays constant
        
        // Calculate impact angle
        double impactAngleRad = Math.atan2(finalVy, finalVx);
        double impactAngleDeg = Math.toDegrees(impactAngleRad);
        
        System.out.println("Launch angle: " + Math.toDegrees(theta) + "°");
        System.out.println("Time of flight: " + t + " seconds");
        System.out.println("Expected height: " + ShooterConstants.TARGET_HEIGHT_DELTA_M + " m");
        System.out.println("Actual height: " + finalY + " m");
        System.out.println("Height error: " + Math.abs(finalY - ShooterConstants.TARGET_HEIGHT_DELTA_M) + " m");
        System.out.println();
        System.out.println("Expected impact angle: " + Math.toDegrees(ShooterConstants.IMPACT_ANGLE_RAD) + "°");
        System.out.println("Actual impact angle: " + impactAngleDeg + "°");
        System.out.println("Angle error: " + Math.abs(impactAngleDeg - Math.toDegrees(ShooterConstants.IMPACT_ANGLE_RAD)) + "°");
        System.out.println();
        
        if (Math.abs(impactAngleDeg - Math.toDegrees(ShooterConstants.IMPACT_ANGLE_RAD)) < 1.0) {
            System.out.println("✓ FORMULA IS CORRECT!");
        } else {
            System.out.println("✗ WARNING: Formula may have errors");
        }
        
        System.out.println("========================================");
    }
}
