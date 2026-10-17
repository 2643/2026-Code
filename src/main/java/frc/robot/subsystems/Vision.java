// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceTags;
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
    // Only track the AprilTags on our own alliance's half of the field.
    AllianceTags.update(primaryLimelight, limelightName1, limelightName2);
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
  // Keep the cameras pointed at our own alliance's tags (no-op unless the alliance changed).
  AllianceTags.update(primaryLimelight, limelightName1, limelightName2);

  // Primary (default) limelight
  fiducialID = LimelightHelpers.getFiducialID(primaryLimelight);
  boolean primaryIsOurs = AllianceTags.isAllianceTag(fiducialID);
  isVisible = LimelightHelpers.getTV(primaryLimelight) && primaryIsOurs;
  yaw = isVisible ? LimelightHelpers.getTX(primaryLimelight) : 0.0;
  area = isVisible ? LimelightHelpers.getTA(primaryLimelight) : 0.0;
  if (!primaryIsOurs) {
    fiducialID = -1;
  }

  SmartDashboard.putBoolean("Has Target (primary)", isVisible);
  SmartDashboard.putNumber("Target Yaw (primary)", yaw);
  SmartDashboard.putNumber("Target Area (primary)", area);
  SmartDashboard.putNumber("Fiducial ID (primary)", fiducialID);

  // Secondary named limelights (if present)
  boolean ll1Visible = LimelightHelpers.getTV(limelightName1)
      && AllianceTags.isAllianceTag(LimelightHelpers.getFiducialID(limelightName1));
  double ll1Yaw = ll1Visible ? LimelightHelpers.getTX(limelightName1) : 0.0;
  double ll1Area = ll1Visible ? LimelightHelpers.getTA(limelightName1) : 0.0;

  boolean ll2Visible = LimelightHelpers.getTV(limelightName2)
      && AllianceTags.isAllianceTag(LimelightHelpers.getFiducialID(limelightName2));
  double ll2Yaw = ll2Visible ? LimelightHelpers.getTX(limelightName2) : 0.0;
  double ll2Area = ll2Visible ? LimelightHelpers.getTA(limelightName2) : 0.0;

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
