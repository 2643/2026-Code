// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.TreeMap;

/** 
 * Lookup table for hub shooting parameters based on distance.
 * Supports linear interpolation between data points.
 */
public class HubLookUpTable {
    
    /** Data structure to hold shooting parameters */
    public static class ShootingParameters {
        public final double shooterSpeed;     // RPS (Revolutions Per Second)
        public final double trajectoryAngle;  // Degrees
        public final double timeOfFlight;     // Seconds
        
        public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }
    
    // TreeMap automatically sorts by distance (key)
    private final TreeMap<Double, ShootingParameters> lookupTable;
    
    public HubLookUpTable() {
        lookupTable = new TreeMap<>();
        initializeLookupTable();
    }
    
    /** Initialize the lookup table with known data points */
    private void initializeLookupTable() {
        // Distance (m), Shooter Speed (RPS), Trajectory Angle (°), Time of Flight (s)
        // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
        // Trajectory angles: 90° = straight up, 45° = maximum distance
        // addEntry(4,  -65, 0.7, 1);
        addEntry(2.86,  -60, 0.45, 1.15);
        addEntry(2.3,  -55, 0.4, 1.18);
        addEntry(4.51,  -65, 1, 1.41);
        addEntry(3.0, -60, 0.625, 1.34);
        addEntry(3.5,  -60, 1.1, 1.2);
        addEntry(4.6,  -65, 1.6, 1.5);
        addEntry(2.5, -55, 0.65, 1.2);
        addEntry(3.2, -60, 0.8, 1.267);

        // AI GENERATED DATA - NOT TESTED
        addEntry(2, -55, 0.301, 1.12);
        addEntry(2.25, -55, 0.401, 1.15);
        addEntry(2.5, -55, 0.502, 1.181);
        addEntry(2.75, -60, 0.603, 1.212);
        addEntry(3, -60, 0.704, 1.243);
        addEntry(3.25, -60, 0.804, 1.274);
        addEntry(3.5, -60, 0.905, 1.304);
        addEntry(3.75, -60, 1.006, 1.335);
        addEntry(4, -65, 1.107, 1.366);
        addEntry(4.25, -65, 1.208, 1.397);
        addEntry(4.5, -65, 1.308, 1.428);
        addEntry(4.75, -65, 1.409, 1.458);
        addEntry(5, -65, 1.51, 1.489);
        //end of AI GENERATED DATA
        
        // Max distance - lowest angle
    }
    
    /** Add an entry to the lookup table */
    public void addEntry(double distance, double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
        lookupTable.put(distance, new ShootingParameters(shooterSpeed, trajectoryAngle, timeOfFlight));
    }
    
    /** 
     * Get interpolated shooting parameters for a given distance 
     * @param distance Distance to target in meters
     * @return Interpolated shooting parameters
     */
    public ShootingParameters getParameters(double distance) {
        // Check if exact match exists
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }
        
        // Get the surrounding values
        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);
        
        // Handle edge cases
        if (lowerKey == null) {
            return lookupTable.get(upperKey); // Below minimum distance
        }
        if (upperKey == null) {
            return lookupTable.get(lowerKey); // Above maximum distance
        }
        
        // Perform linear interpolation
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);
        
        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        
        double interpolatedSpeed = lerp(lower.shooterSpeed, upper.shooterSpeed, ratio);
        double interpolatedAngle = lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio);
        double interpolatedTime = lerp(lower.timeOfFlight, upper.timeOfFlight, ratio);
        
        return new ShootingParameters(interpolatedSpeed, interpolatedAngle, interpolatedTime);
    }
    
    /** Linear interpolation helper */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
    
    /** Get shooter speed for a given distance */
    public double getShooterSpeed(double distance) {
        return getParameters(distance).shooterSpeed;
    }
    
    /** Get trajectory angle for a given distance */
    public double getTrajectoryAngle(double distance) {
        return getParameters(distance).trajectoryAngle;
    }
    
    /** Get time of flight for a given distance */
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }
}