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

  public enum Wheel {
    ON,
    OFF,
  }

  public double wheelSpeed;
  public double targetWheelSpeed;
  public double targetIndexSpeed;
  public double indexSpeed;
  public boolean shoot = false;
  public boolean spin = false;


  TalonFX flyWheel = new TalonFX(Constants.StorageConstants.wheelID);
  TalonFX indexMotor1 = new TalonFX(Constants.StorageConstants.indexMotorID);
  TalonFX indexMotor2 = new TalonFX(Constants.StorageConstants.indexMotor2ID);

  DigitalInput indexLimit = new DigitalInput(Constants.StorageConstants.indexLimitPort);

  private Timer timer = new Timer();
  private Timer flyTimer = new Timer();
  
  Phase currentPhase = Phase.ATTACK;
  Indexer currentIndexer = Indexer.OFF;
  Wheel currentWheel = Wheel.OFF;

  public final VelocityVoltage vel = new VelocityVoltage(0).withSlot(0);
  
  public MotorAlignmentValue MotorAlignment = MotorAlignmentValue.Aligned; // Aligned or Opposed
  TalonFXConfiguration configs = new TalonFXConfiguration();

  public Storage() {
    configs.Slot0.kP = Constants.StorageConstants.wheelP;
    configs.Slot0.kI = Constants.StorageConstants.wheelI;
    configs.Slot0.kD = Constants.StorageConstants.wheelD;
    
    configs.CurrentLimits.StatorCurrentLimit = Constants.StorageConstants.wheelStatorLimit;
    configs.CurrentLimits.SupplyCurrentLimit = Constants.StorageConstants.wheelSupplyLimit;

    flyWheel.getConfigurator().apply(configs);
    indexMotor2.setControl(new Follower(indexMotor1.getDeviceID(), MotorAlignment));
  }

  public double getFlywheelSpeed(){
    return wheelSpeed;
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
  
public void moveWheel(double speed) {
  targetWheelSpeed = speed;
   if (getWheel() == Wheel.ON)
  { 
      flyWheel.setControl(vel.withVelocity(speed).withFeedForward(Constants.StorageConstants.wheelFF));
  }
  else
  {
    flyWheel.setControl(new DutyCycleOut(0));
  }
}

public void moveIndexer(double speed) {
  targetIndexSpeed = speed;
   if (getIndexer() == Indexer.ON)
  { 
      indexMotor1.setControl(new DutyCycleOut(speed));
      
  }
  else
  {
    indexMotor1.setControl(new DutyCycleOut(0));
  }
}

public Phase getPhase() {
  return currentPhase;
}

public void setPhase(Phase phase) {
  currentPhase = phase;
}

public Wheel getWheel() {
  return currentWheel;
}

public void setWheel(Wheel wheel) {
  currentWheel = wheel;
  if (wheel == Wheel.ON){
    spin = true;
  } else{
    spin = false;
  }
}

public void setIndexer(Indexer indexer) {
  currentIndexer = indexer;
  if (indexer == Indexer.ON){
    shoot = true;
  } else{
    shoot = false;
  }
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

    wheelSpeed = flyWheel.getRotorVelocity().refresh().getValueAsDouble();
    indexSpeed = indexMotor1.getRotorVelocity().refresh().getValueAsDouble();

    SmartDashboard.putBoolean("Storage Limit Switch", indexLimit.get());
    SmartDashboard.putString("Phase", currentPhase.toString());
    SmartDashboard.putString("On Off", currentIndexer.toString());
    SmartDashboard.putNumber("Current Wheel Speed", wheelSpeed);
    SmartDashboard.putNumber("Target Wheel Speed", targetWheelSpeed);
    SmartDashboard.putNumber("Current Indexer Speed", indexSpeed);
    SmartDashboard.putNumber("Target Indexer Speed", targetIndexSpeed);
    SmartDashboard.putBoolean("Wheel", spin);
    SmartDashboard.putBoolean("Shooting", shoot);

    moveWheel(SmartDashboard.getNumber("Wheel Speed", 0));
  }
}

