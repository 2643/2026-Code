// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Turret extends SubsystemBase {

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED
  }

  public boolean isLimitedX1;
  public boolean isLimitedX2;
  public boolean isLimitedY1;
  public boolean isLimitedY2;
  public boolean isLocked = false;
  public boolean isVisible;
  public double yaw;
  public double area;
  public double tx;
  public double ty;
  public double fiducialID;
  public double range;
  public double percentOutputValue;
  public static double hoodTarget;
  public static double swivelTarget;
  States currentState = States.INITIALIZING;


  public TalonFX swivelMotor = new TalonFX(Constants.TurretConstants.swivelID);

  public SparkMax hoodMotor = new SparkMax(Constants.TurretConstants.hoodID, MotorType.kBrushless);
  public RelativeEncoder encoder = hoodMotor.getEncoder();
 
  private final String limelightName2 = "limelight";
  private final String limelightURL2 = "http://10.26.43.200:5801/";

  private final String limelightName = "limelight-bhavik";
  private final String limelightURL = "http://10.26.43.201:5801/";

  // public DigitalInput hoodLimit = new DigitalInput(Constants.TurretConstants.hoodLimitPort); //removed bc different initialization method
  public DigitalInput swivelLimit = new DigitalInput(Constants.TurretConstants.swivelLimitPort);

  TalonFXConfiguration configs = new TalonFXConfiguration();
  
  public MAXMotionConfig motorConfig = new MAXMotionConfig();
  public ClosedLoopConfig motorConfigClosed = new ClosedLoopConfig();
  public SparkMaxConfig motorConfigBase = new SparkMaxConfig();
  ClosedLoopConfig revConfig = new ClosedLoopConfig();
  SparkClosedLoopController m_controller = hoodMotor.getClosedLoopController();

  


  public Turret() {
    configs.Slot0.kP = Constants.TurretConstants.swivelP;
    configs.Slot0.kI = Constants.TurretConstants.swivelP;
    configs.Slot0.kD = Constants.TurretConstants.swivelP;
    
    configs.MotionMagic.MotionMagicAcceleration = 50;
    configs.MotionMagic.MotionMagicCruiseVelocity = 50;
  
    swivelMotor.setPosition(0);

    swivelMotor.getConfigurator().apply(configs);
    swivelMotor.setNeutralMode(NeutralModeValue.Coast);
    
    motorConfig.cruiseVelocity(100).maxAcceleration(100);

    motorConfigClosed
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.TurretConstants.hoodP)
      .i(Constants.TurretConstants.hoodI)
      .d(Constants.TurretConstants.hoodD)
      .outputRange(-5, 5)
      .apply(motorConfig);

      motorConfigBase.apply(motorConfigClosed);

    hoodMotor.configure(motorConfigBase, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public States getState() {
    return currentState;
  }

  public void setState(States state) {
    currentState = state;
  }

  public void setPos(double pos) {
      swivelMotor.setPosition(pos);
  }

  public String getLimelightURL() {
    return limelightURL;
  }

  public double getTX() {
    return tx;
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
    // if (tx < 0) {
    //   percentOutputValue = -0.05;
    // } else if (tx > 0) {
    //   percentOutputValue = 0.05;
    // }
    return percentOutputValue;
  }
  // public double fullReverseRotation() {
  //   if (tx > 0) {
  //     percentOutputValue = -(Math.log(tx)/600*5);
  //   }
  //   else if (tx < 0) {
  //     percentOutputValue = Math.log(-tx)/600*5;
  //   }
  //   return percentOutputValue;
  // }
   public void moveSwivel(double position) {
    swivelTarget = position;
    swivelMotor.setControl(new MotionMagicVoltage(swivelTarget));
  }

  public void moveHood(double position) {
    hoodTarget = position;
    m_controller.setSetpoint(hoodTarget, ControlType.kMAXMotionPositionControl);
  }
  public double getHoodPos() {
    return encoder.getPosition();
  }

  public boolean isAtPosition() {
    double encoderPosition = encoder.getPosition();
    return Math.abs(encoderPosition - hoodTarget) <= 1; // MARGIN OF ERROR
  }

  public double getTY() {
    return ty;
  }


  public void autoAlign(){
    if (isVisible == true && tx>0 && isLimitedX1 == false && isLimitedX2 == false && isLocked == false) {
      swivelMotor.setControl(new DutyCycleOut(getPercentOutput()));
    } 
    // else if (isVisible == true && tx>0 && isLimitedX1 == true && isLimitedX2 == true && isLocked == false) {
    //   swivelMotor.setControl(new DutyCycleOut(fullReverseRotation()));
    // }
    else {
      swivelMotor.setControl(new DutyCycleOut(0));
    }
  }

  public void autoPitch() {
    // old
    // Enumeration<Integer> keys = Constants.TurretConstants.areaToAngle.keys();
    // double ta = 1/Math.log(area);
    //   while (keys.hasMoreElements()) {
    //     int key = keys.nextElement(); 
    //     Double angle = Constants.TurretConstants.areaToAngle.get(key);
    //     if (angle == Math.round(ta)) {
    //       m_controller.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    //     }
    //   }
    
    int roundedArea = (int) Math.round(Math.log(1/area));
    Double angle = Constants.TurretConstants.areaToAngle.get(roundedArea);
    if (angle != null) {
      moveHood(angle);
    }
  }

    
  public void downMotorPosX(){
    if (isLimitedX2 == true) {
      moveSwivel(getSwivelPos() - 5);
    }
  }
  public void upMotorPosY(){
    if (isLimitedY2 == true) {
      moveSwivel(getSwivelPos() - 0.5);
    }
  }
  public void downMotorPosY(){
    if (isLimitedY1 == true) {
      moveSwivel(getSwivelPos() + 0.5);
    }
  }
  
  public double getSwivelPos(){
    return swivelMotor.getPosition().getValueAsDouble();
  }
  public void setEncoder() {
    encoder.setPosition(0);
    moveHood(2.9);
  }
  public boolean getSwivelLimit(){
    return swivelLimit.get();
  }

  public void limitX() {
    if (getSwivelPos() >= 3) {
      isLimitedX1 = false;
    }
    else if (getSwivelPos() <= -3) {
      isLimitedX2 = false;
    }
    else {
      isLimitedX1 = false;
      isLimitedX2 = false;
    }
  }
  public void limitY() {
    // if (getHoodPos() >= 2.9) {
    //   isLimitedY1 = true;
    // }
    // else if (getHoodPos() <= 0) {
    //   isLimitedY2 = true;
    // }
    // else {
    //   isLimitedY1 = false;
    //   isLimitedY2 = false;
    // }
  }
  public void setHood(double position) {
    hoodMotor.set(position);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoAlign();
    limitX();
    limitY();

    isVisible = LimelightHelpers.getTV(limelightName);
    // yaw = LimelightHelpers.getTX(limelightName);
    tx = LimelightHelpers.getTY(limelightName);  // Horizontal offset (same as yaw)
    // ty = LimelightHelpers.getTY(limelightName);  // Vertical offset
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
    
    SmartDashboard.putBoolean("Has Target", isVisible);
    SmartDashboard.putNumber("Target Yaw", yaw);
    SmartDashboard.putNumber("Target Hood Position", hoodTarget);
    SmartDashboard.putNumber("Target Swivel Position", swivelTarget);
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Limelight Stream", limelightURL);
    SmartDashboard.putString("Driver Cam", limelightURL2);
    SmartDashboard.putNumber("percentOutputValue", percentOutputValue);
    SmartDashboard.putNumber("Current Swivel Pos", getSwivelPos());
    SmartDashboard.putNumber("Current Hood Pos", getHoodPos());
    SmartDashboard.putBoolean("Swivel Limit", swivelLimit.get());
    SmartDashboard.putString("Current State", currentState.toString());
  }
}