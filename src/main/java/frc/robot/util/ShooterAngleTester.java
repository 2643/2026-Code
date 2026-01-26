// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

/**
 * Utility class for testing the ShooterAngleSolver without a physical robot.
 * Publishes test results to NetworkTables for visualization in Shuffleboard/Elastic.
 */
public class ShooterAngleTester {
    
    /**
     * Runs a comprehensive test of the shooter angle solver across multiple distances.
     * Results are published to SmartDashboard.
     */
    public static void runTestSuite() {
        SmartDashboard.putString("ShooterTest/Status", "Running tests...");
        
        // Test at various distances
        double[] testDistances = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        
        for (double distance : testDistances) {
            double angle = ShooterAngleSolver.solveTheta(
                ShooterConstants.BALL_VELOCITY_MPS,
                distance,
                ShooterConstants.TARGET_HEIGHT_DELTA_M,
                ShooterConstants.IMPACT_ANGLE_RAD
            );
            
            double angleDegrees = ShooterAngleSolver.toDegrees(angle);
            SmartDashboard.putNumber("ShooterTest/Distance_" + distance + "m_Angle", angleDegrees);
        }
        
        SmartDashboard.putString("ShooterTest/Status", "Tests complete!");
    }
    
    /**
     * Tests a single distance and publishes detailed results.
     */
    public static void testSingleDistance(double distance) {
        double angle = ShooterAngleSolver.solveTheta(
            ShooterConstants.BALL_VELOCITY_MPS,
            distance,
            ShooterConstants.TARGET_HEIGHT_DELTA_M,
            ShooterConstants.IMPACT_ANGLE_RAD
        );
        
        double angleDegrees = ShooterAngleSolver.toDegrees(angle);
        
        // Publish results
        SmartDashboard.putNumber("ShooterTest/Input_Distance_M", distance);
        SmartDashboard.putNumber("ShooterTest/Output_Angle_Deg", angleDegrees);
        SmartDashboard.putNumber("ShooterTest/Output_Angle_Rad", angle);
        
        // Also show the constants being used
        SmartDashboard.putNumber("ShooterTest/Velocity_MPS", ShooterConstants.BALL_VELOCITY_MPS);
        SmartDashboard.putNumber("ShooterTest/Height_Delta_M", ShooterConstants.TARGET_HEIGHT_DELTA_M);
        SmartDashboard.putNumber("ShooterTest/Impact_Angle_Deg", Math.toDegrees(ShooterConstants.IMPACT_ANGLE_RAD));
    }
    
    /**
     * Simulates Limelight ta values and shows what angles would result.
     * Useful for understanding the ta-to-angle mapping.
     */
    public static void simulateLimelightMapping() {
        // Simulate ta values (target area percentage)
        // Larger ta = closer, smaller ta = farther
        for (double ta = 0.5; ta <= 10.0; ta += 0.5) {
            double distance = Math.abs(ta) * ShooterConstants.TA_TO_DISTANCE_FACTOR;
            double angle = ShooterAngleSolver.solveTheta(
                ShooterConstants.BALL_VELOCITY_MPS,
                distance,
                ShooterConstants.TARGET_HEIGHT_DELTA_M,
                ShooterConstants.IMPACT_ANGLE_RAD
            );
            
            double angleDegrees = ShooterAngleSolver.toDegrees(angle);
            SmartDashboard.putNumber("LimelightSim/TA_" + ta + "_Angle", angleDegrees);
        }
    }
}
