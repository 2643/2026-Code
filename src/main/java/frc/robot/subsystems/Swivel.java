// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Swivel extends SubsystemBase {

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED
  }

  public static TalonFX swivelMotor = new TalonFX(0); 
  public DigitalInput swivelLimit = new DigitalInput(Constants.TurretConstants.swivelLimitPort);
  public static double swivelTarget;
  public double percentOutputValue;
  public double tx;
  private final String limelightName = "limelight-bhavik";
  private final String limelightURL = "http://10.26.43.201:5801/";
  public boolean isVisible;
  public double yaw;
  public double area;
  public double fiducialID;
  States currentState = States.INITIALIZING;

  
  TalonFXConfiguration configs = new TalonFXConfiguration();

 public Swivel() {
    var slot0config = configs.Slot0;
    var magicmotionconfig = configs.MotionMagic;
    configs.Slot0.kP = Constants.TurretConstants.swivelP;
    configs.Slot0.kI = Constants.TurretConstants.swivelI;
    configs.Slot0.kD = Constants.TurretConstants.swivelD;

    magicmotionconfig.MotionMagicAcceleration = 40;
    magicmotionconfig.MotionMagicCruiseVelocity = 40;
    swivelMotor.getConfigurator().apply(configs);

    setSwivelPos(0);
    
  }

   public void moveSwivel(double target) {
    swivelTarget = target;
    swivelMotor.setControl(new MotionMagicVoltage(target));
        System.out.println("hi2");

  }

  public States getState() {
    return currentState;
  }

  public void setState(States state) {
    currentState = state;
  }
  
  public double getSwivelPos(){
    return swivelMotor.getPosition().getValueAsDouble();
  }

  public void setSwivelPos(double pos) {
    swivelMotor.setPosition(pos);
  }

  public boolean getSwivelLimit(){
    return swivelLimit.get();
  }

    public double getPercentOutput() {
    if (tx > 0) {
      percentOutputValue = Math.log(tx)/600*5/2*8;
      if (percentOutputValue >= 0.2) {
        percentOutputValue = 0.2;
      }
    }
    else if (tx < 0) {
      percentOutputValue = -(Math.log(-tx)/600/2*5*8);
      if (percentOutputValue <= -0.2) {
        percentOutputValue = -0.2;
      }
    }
    return percentOutputValue;
  }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run  s123
    tx = LimelightHelpers.getTY(limelightName);  // Horizontal offset (same as yaw)
    isVisible = LimelightHelpers.getTV(limelightName);
    // yaw = LimelightHelpers.getTX(limelightName);
    // ty = LimelightHelpers.getTY(limelightName);  // Vertical offset
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
    SmartDashboard.putNumber("Current Swivel Pos", getSwivelPos());
    SmartDashboard.putNumber("Target Swivel Position", swivelTarget);
    SmartDashboard.putNumber("percentOutputValue", percentOutputValue);
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putString("Current State", currentState.toString());
    SmartDashboard.putBoolean("Swivel Limit", getSwivelLimit());

    //  if (getSwivelPos() > Constants.TurretConstants.swivelSoftLimit1) {
    //   moveSwivel(Constants.TurretConstants.swivelSoftLimit1 - 0.1);
    // } else if (getSwivelPos() < Constants.TurretConstants.hoodSoftLimit2) {
    //   moveSwivel(Constants.TurretConstants.swivelSoftLimit2 + 0.1);
    // } else if (getSwivelPos() >= Constants.TurretConstants.swivelHardLimit1 || getSwivelPos() <= Constants.TurretConstants.swivelHardLimit2) {
    //   swivelMotor.disable();
    // }
  }
}
