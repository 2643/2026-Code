// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
  public boolean isVisible;
  public double yaw;
  public double range;
  public double area;
  public double fiducialID;
    
  // private final String limelightName = "limelight";

  // Names/URLs for limelights
  private final String primaryLimelight = "limelight-bhavik";

  // Additional named Limelight cameras on the robot (HTTP streams)
  private final String limelightName1 = "limelight-allen";
  private final String limelightURL1 = "http://10.26.43.200:5801/";

  private final String limelightName2 = "limelight-bhavik";
  private final String limelightURL2 = "http://10.26.43.201:5801/";

  // public final PhotonCamera camera = new PhotonCamera("placeholder");
 
  /**NOTES:
   * In the future prolly make a state machine that has the different states of each different scoring method
   * (for reefscape: the L#s/Algae)
   * This is so that when we do our vision logic we can just see what scoring that we are doing and then automatically do robot functions depending on the state (doesn't have to be in this file)
   */


  /** Creates a new Vision. */
  public Vision() {
  LimelightHelpers.setLEDMode_PipelineControl(primaryLimelight);
    // Start automatic MJPEG captures for both Limelight HTTP streams so dashboards
    // (Shuffleboard/SmartDashboard/CameraServer/Elastic) can view them.
    // Try starting HTTP camera captures. Some dashboards need an explicit
    // HttpCamera with a proper MJPEG stream path. Try a few common endpoints.
    try {
      var cam1 = new HttpCamera(limelightName1, limelightURL1 + "stream.mjpg");
      CameraServer.startAutomaticCapture(cam1);
    } catch (Exception e1) {
      try {
        var cam1b = new HttpCamera(limelightName1, limelightURL1 + "stream");
        CameraServer.startAutomaticCapture(cam1b);
      } catch (Exception ignored) {
        try {
          var cam1c = new HttpCamera(limelightName1, limelightURL1);
          CameraServer.startAutomaticCapture(cam1c);
        } catch (Exception ignored2) {
        }
      }
    }

    try {
      var cam2 = new HttpCamera(limelightName2, limelightURL2 + "stream.mjpg");
      CameraServer.startAutomaticCapture(cam2);
    } catch (Exception e2) {
      try {
        var cam2b = new HttpCamera(limelightName2, limelightURL2 + "stream");
        CameraServer.startAutomaticCapture(cam2b);
      } catch (Exception ignored) {
        try {
          var cam2c = new HttpCamera(limelightName2, limelightURL2);
          CameraServer.startAutomaticCapture(cam2c);
        } catch (Exception ignored2) {
        }
      }
    }
  }


  
  public void updateData() {
  // Primary (default) limelight
  isVisible = LimelightHelpers.getTV(primaryLimelight);
  yaw = LimelightHelpers.getTX(primaryLimelight);
  area = LimelightHelpers.getTA(primaryLimelight);
  fiducialID = LimelightHelpers.getFiducialID(primaryLimelight);

  SmartDashboard.putBoolean("Has Target (primary)", isVisible);
  SmartDashboard.putNumber("Target Yaw (primary)", yaw);
  SmartDashboard.putNumber("Target Area (primary)", area);
  SmartDashboard.putNumber("Fiducial ID (primary)", fiducialID);

  // Secondary named limelights (if present)
  boolean ll1Visible = LimelightHelpers.getTV(limelightName1);
  double ll1Yaw = LimelightHelpers.getTX(limelightName1);
  double ll1Area = LimelightHelpers.getTA(limelightName1);

  boolean ll2Visible = LimelightHelpers.getTV(limelightName2);
  double ll2Yaw = LimelightHelpers.getTX(limelightName2);
  double ll2Area = LimelightHelpers.getTA(limelightName2);

  SmartDashboard.putBoolean("Has Target (allen)", ll1Visible);
  SmartDashboard.putNumber("Target Yaw (allen)", ll1Yaw);
  SmartDashboard.putNumber("Target Area (allen)", ll1Area);

  SmartDashboard.putBoolean("Has Target (bhavik)", ll2Visible);
  SmartDashboard.putNumber("Target Yaw (bhavik)", ll2Yaw);
  SmartDashboard.putNumber("Target Area (bhavik)", ll2Area);
  }


  @Override
  public void periodic() {
  updateData();
    // This method will be called once per scheduler run
  }
}
