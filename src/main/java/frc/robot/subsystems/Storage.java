// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Storage extends SubsystemBase {

  public enum Phase {
    ATTACK,
    DEFENSE,
  }
  public enum Indexer {
    ON,
    OFF,
  }

  public boolean detected = false;
  public double flyWheelSpeed;

  TalonFX flyWheel = new TalonFX(Constants.StorageConstants.wheelPort);
  TalonFX indexMotor1 = new TalonFX(Constants.StorageConstants.indexMotorID);
  TalonFX indexMotor2 = new TalonFX(Constants.StorageConstants.indexMotor2ID);

  DigitalInput indexLimit = new DigitalInput(Constants.StorageConstants.indexLimitPort);

  private Timer timer = new Timer();
  private Timer flyTimer = new Timer();
  
  Phase currentPhase = Phase.ATTACK;
  Indexer currentIndexer = Indexer.OFF;

  public final VelocityVoltage vel = new VelocityVoltage(0).withSlot(0);
  
  public MotorAlignmentValue MotorAlignment = MotorAlignmentValue.Aligned; // Aligned or Opposed
  TalonFXConfiguration configs = new TalonFXConfiguration();

  public Storage() {
    configs.Slot0.kP = Constants.StorageConstants.wheelP;
    configs.Slot0.kI = Constants.StorageConstants.wheelI;
    configs.Slot0.kD = Constants.StorageConstants.wheelD;
    

    flyWheel.getConfigurator().apply(configs);
    indexMotor2.setControl(new Follower(indexMotor1.getDeviceID(), MotorAlignment));
  }
  public void getFlywheelSpeed(){
    flyWheelSpeed = flyWheel.getRotorVelocity().refresh().getValueAsDouble();
  }
  public void delayMotorStart(){
    if (getIndexer() == Indexer.ON) {
      timer.start();
      if (timer.hasElapsed(3)) {
        resetTimer();
        indexMotor1.setControl(new DutyCycleOut(Constants.StorageConstants.indexSpeed));
      } 
    } else {
      indexMotor1.setControl(new DutyCycleOut(0));
    }
  }
  
public void moveMotor(double speed) {
   if (getIndexer() == Indexer.ON)
  { 
    // flyWheel.setControl(new DutyCycleOut(speed));
    // currentSpeed = speed;
    // flyTimer.start();
    //   if (flyTimer.hasElapsed(5)) {
    //     flyWheel.setControl(new DutyCycleOut(speed-0.1));
    //   } if (flyTimer.hasElapsed(7)) {
    //     flyWheel.setControl(new DutyCycleOut(speed-0.15));
    //   } if (flyTimer.hasElapsed(9)) {
    //     flyWheel.setControl(new DutyCycleOut(speed));
    //   } if (flyTimer.hasElapsed(11)) {
    //     flyWheel.setControl(new DutyCycleOut(speed+0.1));
        // resetFlyTimer();
      // } 
      flyWheel.setControl(vel.withVelocity(speed).withFeedForward(Constants.StorageConstants.wheelFF));
  }
  else
  {
    flyWheel.setControl(new DutyCycleOut(0));
  }
}

public Phase getPhase() {
  return currentPhase;
}

public void setPhase(Phase phase) {
  currentPhase = phase;
}

public void setIndexer(Indexer indexer) {
  currentIndexer = indexer;
}

public Indexer getIndexer() {
  return currentIndexer;
}


public void resetTimer(){
  timer.stop();
  timer.reset();
}
public void resetFlyTimer(){
  flyTimer.stop();
  flyTimer.reset();
}
  @Override
  public void periodic() {

    delayMotorStart();
    getFlywheelSpeed();

    // if("limit switch", indexLimit.get()) {
    //   detected = true;
    //   lastDetectionTime = timer.get();
    // }
    // else {
    //   detected = false;
    // }

    // if (timer.get() - lastDetectionTime >= 5.0) {
    //   moveMotor(0);
    // }
    SmartDashboard.putBoolean("Storage Limit Switch", indexLimit.get());
    SmartDashboard.putBoolean("Detected", detected);
    SmartDashboard.putString("Phase", currentPhase.toString());
    SmartDashboard.putString("On/Off", currentIndexer.toString());
    SmartDashboard.putNumber("Current Wheel Speed", flyWheelSpeed);

  }
}

