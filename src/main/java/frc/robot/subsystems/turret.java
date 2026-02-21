// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;




import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Turret extends SubsystemBase {
  public double target = 0;
  public double pos;
  public boolean isLimitedX1;
  public boolean isLimitedX2;
  public boolean isVisible;
  public double yaw;
  public double area;
  public double tx;
  public double ty;
  public double fiducialID;
  public double range;
  public double percentOutputValue;
  public double targetPosition;
  public double p = 2.0;
  public double i = 1.0;
  public double d = 0.0;
  private final String limelightName = "limelight";
  private final String limelightURL = "http://10.26.43.200:5801/";
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  public SparkMax hoodMotor = new SparkMax(9, MotorType.kBrushless);
  public RelativeEncoder encoder = hoodMotor.getEncoder();
  public SparkMaxConfig motorConfig;


  ClosedLoopConfig revConfig = new ClosedLoopConfig();
  SparkClosedLoopController m_controller = hoodMotor.getClosedLoopController();
  public Turret() {
    var motionmagicconfigs = configs.MotionMagic;
    var slot0configs = configs.Slot0;
  
    slot0configs.kP = 13;
    slot0configs.kI = 0;
    slot0configs.kD = 0;
  
    motionmagicconfigs.MotionMagicAcceleration = 20;
    motionmagicconfigs.MotionMagicCruiseVelocity = 20;
  
    motorX.getConfigurator().apply(configs);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(p)
      .i(i)
      .d(d)
      .outputRange(-1, 1);

      

  }
  /** Creates a new turretx. */
  TalonFX motorX = new TalonFX(4);
  TalonFXConfiguration configs = new TalonFXConfiguration();
  DigitalInput limitX = new DigitalInput(0);
  DigitalInput limitY = new DigitalInput(1);
  




  public void position() {
      
      motorX.setPosition(0);
  }
    public void NeoMotorPosition(double p, double i, double d, double ff) {
        double targetRPM = 3000;
    }
    


  public String getLimelightURL() {
    return limelightURL;
  }



  public double getTX() {
    return tx;
  }
  public double getPercentOutput() {
    if (tx > 0) {
      percentOutputValue = Math.log(tx)/600*5;
      if (percentOutputValue >= 0.2) {
        percentOutputValue = 0.2;
      }
    }
    else if (tx < 0) {
      percentOutputValue = -(Math.log(-tx)/600*5);
      if (percentOutputValue <= -0.2) {
        percentOutputValue = -0.2;
      }
    }
    return percentOutputValue;
  }
  public double fullReverseRotation() {
    if (tx > 0) {
      percentOutputValue = -(Math.log(tx)/600*5);
    }
    else if (tx < 0) {
      percentOutputValue = Math.log(-tx)/600*5;
    }
    return percentOutputValue;
  }
  public void goToPosition(double position) {
    targetPosition = position;
    m_controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public boolean isAtPosition() {
    double encoderPosition = encoder.getPosition();
    return Math.abs(encoderPosition - targetPosition) <= 1; // MARGIN OF ERROR
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
    
    SmartDashboard.putNumber("TurretManualPosition", 90);
    SmartDashboard.putBoolean("Has Target", isVisible);
    SmartDashboard.putNumber("Target Yaw", yaw);
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Limelight Stream", limelightURL);
    SmartDashboard.putNumber("Position", currentPosX());
  }
  public void autoAlign(){
    if (isVisible == true && tx>0 && isLimitedX1 == false && isLimitedX2 == false) {
      motorX.setControl(new DutyCycleOut(getPercentOutput()));
    } 
    else if (isVisible == true && tx>0 && isLimitedX1 == true && isLimitedX2 == true) {
      motorX.setControl(new DutyCycleOut(fullReverseRotation()));
    }
    else {
      motorX.setControl(new DutyCycleOut(0));
    }
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
  
  public double currentPosX(){
    return motorX.getPosition().getValueAsDouble();
  }
  public boolean getLimitX(){
    return limitX.get();
  }
  public boolean getLimitY(){
    return limitY.get();
  }
  public void limit() {
    if (currentPosX() >= 3) {
      isLimitedX1 = true;
    }
    else if (currentPosX() <= -3) {
      isLimitedX2 = true;
    }
    else {
      isLimitedX1 = false;
      isLimitedX2 = false;
    }
  }
  public void setHoodMotor(double position) {
    hoodMotor.set(position);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateData();
    autoAlign();
    limit();
    SmartDashboard.putNumber("TurretPosition", encoder.getPosition());

    double manualPosition = SmartDashboard.getNumber("TurretManualPosition", 90);
    goToPosition(manualPosition);
  }
}
