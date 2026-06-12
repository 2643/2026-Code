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
        // Imported/cleaned from live CSV logs (median, 0.05 m bins):
        // Source CSV lines (timestamp,dist_m,shooter_rps,hood_deg,time_of_flight)
        // 2026-06-08T21:54:08...,2.115,-60.000,0.400,1.180
        // 2026-06-08T21:55:11...,2.235,-60.000,0.400,1.180
        // 2026-06-08T21:42:01...,2.352,-60.000,0.465,1.185
        // 2026-06-08T21:42:45...,2.470,-60.000,0.613,1.197
        // 2026-06-08T21:54:54...,2.539,-60.537,0.629,1.195
        // 2026-06-08T21:42:58...,2.691,-62.659,0.544,1.173

    // addEntry(2.86,  -60-0, 0.45, 1.15);
    // addEntry(2.3,  -65-0, 0.4, 1.18);
    // addEntry(4.51,  -65-0, 1, 1.41);
    // addEntry(3.0, -60-0, 0.625, 1.34);
    // addEntry(3.5,  -60-0, 1.1, 1.2);
    // addEntry(4.6,  -65-0, 1.6, 1.5);
    // addEntry(2.5, -65-0, 0.65, 1.2);
    // addEntry(3.2, -60-0, 0.8, 1.267);
    // addEntry(3.8, -60-0, 0.7, 1.5);
    // addEntry(4.7, -65-0, 1.7, 1.67);
    // addEntry(4, -65-0, 1.25, 1.45);
        // addEntry(, , , 0);
        addEntry(1.6, -55, 0.2, 0);
        addEntry(2.04, -57, 0.45, 0);
        addEntry(2.67, -60, 0.52, 0);
        addEntry(3.06, -63, 0.55, 0);
        addEntry(2.227, -58, 0.34, 0);
        addEntry(0.65, -50, 0, 0);
        addEntry(1.29, -52, 0.2, 0);
        addEntry(2.97, -62, 0.48, 0);
        addEntry(2.5, -53, 0.43, 0);
        addEntry(2.79, -60, 0.7, 0);
        addEntry(1.28, 0, -50, 0);
        addEntry(1.68, 0.1, -54, 0);
        
        // AI GENERATED DATA - NOT TESTED
        // addEntry(2, -05, 0.301, 1.12);
        
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

    /**
     * Replace the current lookup table with new data. The map should be keyed by
     * distance (meters).
     */
    public synchronized void importData(java.util.Map<Double, ShootingParameters> data) {
        lookupTable.clear();
        // Insert sorted by key
        var sorted = new java.util.TreeMap<Double, ShootingParameters>(data);
        lookupTable.putAll(sorted);
    }

    /**
     * Parse a CSV of lines timestamp,dist_m,shooter_rps,hood_deg,time_of_flight and
     * aggregate entries by rounded distance (0.05 m bins) using median values to
     * reduce outliers.
     */
    public synchronized void importFromCsv(java.nio.file.Path csvPath) throws java.io.IOException {
        java.util.List<String> lines = java.nio.file.Files.readAllLines(csvPath);
        java.util.Map<Double, java.util.List<ShootingParameters>> groups = new java.util.HashMap<>();
        for (String line : lines) {
            line = line.trim();
            if (line.isEmpty())
                continue;
            if (line.startsWith("timestamp"))
                continue;
            String[] parts = line.split(",");
            if (parts.length < 5)
                continue;
            try {
                double dist = Double.parseDouble(parts[1]);
                double shooter = Double.parseDouble(parts[2]);
                double angle = Double.parseDouble(parts[3]);
                double tof = Double.parseDouble(parts[4]);
                // Round distance into 5cm bins to group nearby samples
                double bin = Math.round(dist * 20.0) / 20.0; // 0.05m bins
                groups.computeIfAbsent(bin, k -> new java.util.ArrayList<>()).add(new ShootingParameters(shooter, angle, tof));
            } catch (NumberFormatException ex) {
                // ignore malformed
            }
        }

        java.util.Map<Double, ShootingParameters> aggregated = new java.util.HashMap<>();
        for (var e : groups.entrySet()) {
            double key = e.getKey();
            var list = e.getValue();
            java.util.List<Double> shooters = new java.util.ArrayList<>();
            java.util.List<Double> angles = new java.util.ArrayList<>();
            java.util.List<Double> tofs = new java.util.ArrayList<>();
            for (ShootingParameters p : list) {
                shooters.add(p.shooterSpeed);
                angles.add(p.trajectoryAngle);
                tofs.add(p.timeOfFlight);
            }
            java.util.Collections.sort(shooters);
            java.util.Collections.sort(angles);
            java.util.Collections.sort(tofs);
            double shooterMed = shooters.get(shooters.size() / 2);
            double angleMed = angles.get(angles.size() / 2);
            double tofMed = tofs.get(tofs.size() / 2);
            aggregated.put(key, new ShootingParameters(shooterMed, angleMed, tofMed));
        }

        importData(aggregated);
    }

    /** Dump the current lookup table into Java addEntry(...) lines. */
    public synchronized String dumpAsJava() {
        StringBuilder sb = new StringBuilder();
        for (var e : lookupTable.entrySet()) {
            double d = e.getKey();
            var p = e.getValue();
            sb.append(String.format("addEntry(%.3f, %.3f, %.3f, %.3f);\n", d, p.shooterSpeed, p.trajectoryAngle, p.timeOfFlight));
        }
        return sb.toString();
    }
}