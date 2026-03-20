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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.subsystems.Swivel.States;
import frc.robot.util.LimelightHelpers;

public class Hood extends SubsystemBase {
  public boolean disable = false;
  public boolean isLocked = false;
  public boolean isVisible;
  public double yaw;
  public double area;
  public double ty;
  public double fiducialID;
  public double range;
  public double roundedArea;
  public static double hoodTarget;
  public double angle = -1;
  private Timer timer = new Timer();
  public boolean reset = false;
  public double slope = 1.3869;
  public double offset = 1.13255;

  public SparkMax hoodMotor = new SparkMax(Constants.TurretConstants.hoodID, MotorType.kBrushless);
  public RelativeEncoder encoder = hoodMotor.getEncoder();
 
  // private final String limelightName2 = "limelight";
  // private final String limelightURL2 = "http://10.26.43.200:5801/";
   private final String limelightName = "limelight-bhavik";
  private final String limelightURL = "http://10.26.43.201:5801/";

  // public DigitalInput hoodLimit = new DigitalInput(Constants.TurretConstants.hoodLimitPort); //removed bc different initialization method

  TalonFXConfiguration configs = new TalonFXConfiguration();
  
  public MAXMotionConfig motorConfig = new MAXMotionConfig();
  public ClosedLoopConfig motorConfigClosed = new ClosedLoopConfig();
  public SparkMaxConfig motorConfigBase = new SparkMaxConfig();
  ClosedLoopConfig revConfig = new ClosedLoopConfig();
  SparkClosedLoopController m_controller = hoodMotor.getClosedLoopController();

  


  public Hood() {
    motorConfig.cruiseVelocity(Constants.TurretConstants.hoodVel).maxAcceleration(Constants.TurretConstants.hoodAccel);

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
  
  
  // public double fullReverseRotation() {
  //   if (tx > 0) {
  //     percentOutputValue = -(Math.log(tx)/600*5);
  //   }
  //   else if (tx < 0) {
  //     percentOutputValue = Math.log(-tx)/600*5;
  //   }
  //   return percentOutputValue;
  // }
  //  public void moveSwivel(double position) {
  //   swivelTarget = position;
  //   swivelMotor.setControl(new MotionMagicVoltage(swivelTarget));
  // }

  public void moveHood(double position) {
      System.out.println("Moving hood to position: " + position);
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
  
  public void autoPitch() {
    if(RobotContainer.m_Storage.getPhase() == Phase.ATTACK) {
    roundedArea = Math.log(1/area);
    angle = (slope*roundedArea)-offset;
    if (angle < Constants.TurretConstants.hoodSoftLimit1 && angle > Constants.TurretConstants.hoodSoftLimit2) {
      moveHood(angle);
    }
  } else {
    moveHood(2.7);
  }
  }


  public void setEncoder() {
    reset = false;
    encoder.setPosition(0);
    moveHood(2.9);
    moveHood(Constants.TurretConstants.hoodSoftLimit1);
  }

//hello sigmas, this is joshua, I am now in the code mwahhahahahh (signed 3/9/2026)
  public void setHood(double position) {
    hoodMotor.set(position);
  }

  public void startTimer() {
      timer.start();
  }
  
  public void resetTimer() {
    timer.stop();
    timer.reset();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);

    // if(RobotContainer.m_Swivel.getState() == States.INITIALIZED) {
    //  if (getHoodPos() > Constants.TurretConstants.hoodSoftLimit1) {
    //   moveHood(Constants.TurretConstants.hoodSoftLimit1 - 0.1);
    // } else if (getHoodPos() < Constants.TurretConstants.hoodSoftLimit2) {
    //   moveHood(Constants.TurretConstants.hoodSoftLimit2 + 0.1);
    // } else if (getHoodPos() >= Constants.TurretConstants.hoodHardLimit1 || getHoodPos() <= Constants.TurretConstants.hoodHardLimit2) {
    //   // disable = true;
    // }
    // }
    
   if (RobotContainer.m_Swivel.getState() == States.INITIALIZED && timer.hasElapsed(3)){
    moveHood(SmartDashboard.getNumber("Target Hood Position", hoodTarget));
    slope = SmartDashboard.getNumber("Slope", 1.3869);
    offset = SmartDashboard.getNumber("Offset", 1.13255);
    if (!reset){
      reset = true;
      moveHood(1);
    }
    // autoPitch();
  }
    
    SmartDashboard.putNumber("Target Hood Position", hoodTarget);
    SmartDashboard.putNumber("Target Area", area);
    SmartDashboard.putNumber("Fiducial ID", fiducialID);
    SmartDashboard.putString("Driver Cam", limelightURL);
    SmartDashboard.putNumber("Current Hood Position", getHoodPos());
    SmartDashboard.putNumber("Rounded Area", roundedArea);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Offset", offset);
    SmartDashboard.putNumber("Slope", slope);
  }
}