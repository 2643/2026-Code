package frc.robot.util;

/**
 * Quick standalone test to manually verify the formula
 */
public class QuickFormulaTest {
    public static void main(String[] args) {
        System.out.println("Testing Shooter Angle Formula (Binary Search)");
        System.out.println("==============================================\n");
        
        // Test parameters - TESTING DIFFERENT VALUES
        double v0 = 6.5; // m/s (testing - slower for lower angles)
        double dy = 0.772; // m height difference
        double phi = Math.toRadians(-65); // -65 degrees impact angle (testing - steeper)
        
        System.out.println("Constants:");
        System.out.println("  Velocity: " + v0 + " m/s");
        System.out.println("  Height delta: " + dy + " m");
        System.out.println("  Impact angle: " + Math.toDegrees(phi) + "°\n");
        
        System.out.println("Distance | Launch Angle | Verify Impact");
        System.out.println("---------|--------------|---------------");
        
        double targetTanPhi = Math.tan(phi);
        
        for (double dist = 0.5; dist <= 4.0; dist += 0.5) {
            // Binary search
            double thetaLow = 0.0;
            double thetaHigh = Math.PI / 2.0 - 0.01;
            
            for (int i = 0; i < 100; i++) {
                double theta = (thetaLow + thetaHigh) / 2.0;
                double cosTheta = Math.cos(theta);
                double sinTheta = Math.sin(theta);
                double t = dist / (v0 * cosTheta);
                double vy = v0 * sinTheta - 9.81 * t;
                double vx = v0 * cosTheta;
                double actualTanPhi = vy / vx;
                
                if (actualTanPhi < targetTanPhi) {
                    thetaHigh = theta;
                } else {
                    thetaLow = theta;
                }
                
                if (Math.abs(thetaHigh - thetaLow) < 0.0001) break;
            }
            
            double theta = (thetaLow + thetaHigh) / 2.0;
            double thetaDeg = Math.toDegrees(theta);
            
            // Verify the impact angle
            double t = dist / (v0 * Math.cos(theta));
            double vy = v0 * Math.sin(theta) - 9.81 * t;
            double vx = v0 * Math.cos(theta);
            double impactDeg = Math.toDegrees(Math.atan2(vy, vx));
            
            System.out.printf("  %.1f m  |    %.1f°     |    %.1f°%n", dist, thetaDeg, impactDeg);
        }
    }
}
