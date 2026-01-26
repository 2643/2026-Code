// Quick test to see shooter angle calculations
package frc.robot.util;

public class QuickTest {
    public static void main(String[] args) {
        // REAL MEASUREMENTS from your robot!
        double v0 = 9.8;  // TRUE exit velocity (total, not just X!) at 75% power (MEASURED)
        double dy = 0.772;   // Height delta m (Target 49.5" - Shooter 19.1" = 30.4" = 0.772m) (MEASURED!)
        double phi = Math.toRadians(-47);  // Impact angle (BALANCED to get 49-74°)
        
        System.out.println("=== SHOOTER ANGLE TEST RESULTS ===");
        System.out.println("=== REAL ROBOT MEASUREMENTS ===");
        System.out.println("Ball Velocity: " + v0 + " m/s (" + String.format("%.1f", v0 * 2.237) + " mph) @ 75% power [MEASURED - TRUE VELOCITY]");
        System.out.println("Target Height: 49.5 inches (1.257m) [MEASURED]");
        System.out.println("Shooter Exit: 19.1 inches (0.485m) [MEASURED]");
        System.out.println("Height Delta: " + dy + " m [CALCULATED]");
        System.out.println("Impact Angle: " + Math.toDegrees(phi) + " degrees [OPTIMIZED]");
        System.out.println("\nHood angle limits: 49-74 degrees");
        System.out.println("Shooting range: 0.5m - 3.0m");
        System.out.println("\n✓ Using REAL measurements - TRUE exit velocity (not just X-component)!");
        System.out.println("\nDistance -> Hood Angle:");
        System.out.println("------------------------");
        
        double[] testDistances = {0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0};
        
        int inRangeCount = 0;
        
        for (double distance : testDistances) {
            double angle = ShooterAngleSolver.solveTheta(v0, distance, dy, phi);
            double angleDegrees = Math.toDegrees(angle);
            
            String status = "";
            if (angleDegrees < 49) status = " [BELOW MIN]";
            else if (angleDegrees > 74) status = " [ABOVE MAX]";
            else {
                status = " [OK]";
                inRangeCount++;
            }
            
            System.out.printf("%.2fm -> %.2f degrees %s\n", distance, angleDegrees, status);
        }
        
        System.out.println("------------------------");
        System.out.println("RESULT: " + inRangeCount + " out of " + testDistances.length + " distances in range (49-74°)");
        if (inRangeCount >= testDistances.length / 2) {
            System.out.println("✓ GOOD COVERAGE! System should work well.");
        } else {
            System.out.println("⚠ Limited coverage. May need to adjust IMPACT_ANGLE_RAD.");
        }
    }
}
