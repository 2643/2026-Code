// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Turret extends SubsystemBase {
  /** Creates a new turretx. */
  TalonFX motorY = new TalonFX(1);
  TalonFX motorX = new TalonFX(4);
  TalonFXConfiguration configs = new TalonFXConfiguration();
  DigitalInput limitX = new DigitalInput(0);
  DigitalInput limitY = new DigitalInput(1);
  public double target = 0;
  public double pos;

  public boolean isVisible;
  public double yaw;
  public double area;
  public double tx;
  public double ty;
  public double fiducialID;
  public double range;
  private final String limelightName = "limelight";
  private final String limelightURL = "http://10.26.43.200:5801/";
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  public void position() {
      var motionmagicconfigs = configs.MotionMagic;
      var slot0configs = configs.Slot0;
  
      slot0configs.kP = 13;
      slot0configs.kI = 0;
      slot0configs.kD = 0;
  
      motionmagicconfigs.MotionMagicAcceleration = 20;
      motionmagicconfigs.MotionMagicCruiseVelocity = 20;
  
      motorX.getConfigurator().apply(configs);
      motorY.getConfigurator().apply(configs);
      motorX.setPosition(0);
      motorY.setPosition(0);
  }
  public String getLimelightURL() {
    return limelightURL;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }
  public void updateData() {
    isVisible = LimelightHelpers.getTV(limelightName);
    yaw = LimelightHelpers.getTX(limelightName);
    tx = LimelightHelpers.getTX(limelightName);  // Horizontal offset (same as yaw)
    ty = LimelightHelpers.getTY(limelightName);  // Vertical offset
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
        
    SmartDashboard.putBoolean("Has Target", isVisible);
    SmartDashboard.putNumber("Target Yaw", yaw);
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Limelight Stream", limelightURL);
  }
  public void autoAlign(){
    if (isVisible == true && tx>0) {
      motorX.setControl(new DutyCycleOut((Math.log(tx)/600*5)));
    } else if (isVisible == true && tx<0) {
      motorX.setControl(new DutyCycleOut(-(Math.log(-tx)/600*5it )));
    } 
    else {
      motorX.setControl(new DutyCycleOut(0));
    }
  }
  public void moveToPosY(double target){
    target = pos;
    motorY.setControl(motion.withPosition(target));
  }
  public void moveToPosX(double target){
    target = pos;
    motorX.setControl(motion.withPosition(target));
  }
  public void upMotorPosX(){
    moveToPosX(currentPosX() + 5);
  }
  public void downMotorPosX(){
    moveToPosX(currentPosX() - 5);
  }
  public void upMotorPosY(){
    moveToPosY(currentPosY() + 5);
  }
  public void downMotorPosY(){
    moveToPosY(currentPosY() - 5);
  }
  public double currentPosY(){
    return motorY.getPosition().getValueAsDouble();
  }
  public double currentPosX(){
    return motorX.getPosition().getValueAsDouble();
  }
  public boolean getLimitX(){
    return limitX.get();
  }
  public boolean getLimitY(){
    return limitY.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateData();
    autoAlign();
  }
}
