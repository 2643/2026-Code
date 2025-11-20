// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public boolean isVisible;
  public double yaw;
  public double range;

  public final PhotonCamera camera = new PhotonCamera("placeholder");
 
  /**NOTES:
   * In the future prolly make a state machine that has the different states of each different scoring method
   * (for reefscape: the L#s/Algae)
   * This is so that when we do our vision logic we can just see what scoring that we are doing and then automatically do robot functions depending on the state (doesn't have to be in this file)
   */


  /** Creates a new Vision. */
  public Vision() {
  }


  public void autoAlign() {
      boolean targetVisible = false;
      double targetYaw = 0.0;
      double targetRange = 0.0;
      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()) {

          var result = results.get(results.size() - 1);

          if (result.hasTargets()) {

              for (var target : result.getTargets()) {

                  if (target.getFiducialId() == 7) {

                      // Found Tag 7, record its information

                      targetYaw = target.getYaw();

                      targetRange =

                              PhotonUtils.calculateDistanceToTargetMeters(

                                      0.5, // Measured with a tape measure, or in CAD.

                                      1.435, // From 2024 game manual for ID 7

                                      Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.

                                      Units.degreesToRadians(target.getPitch()));


                      targetVisible = true;

                  }

              }
          }
      }

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
