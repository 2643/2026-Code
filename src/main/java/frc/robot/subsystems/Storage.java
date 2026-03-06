// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Storage extends SubsystemBase {
  public double currentSpeed = 0;
  public boolean detected = false;
  public boolean run = false;
  private Timer timer = new Timer();
  
  // private double lastDetectionTime = 0;
  Phase currentPhase = Phase.ATTACK;
  Indexer currentIndexer = Indexer.OFF;
  TalonFX flyWheel = new TalonFX(Constants.StorageConstants.flyWheel);
  TalonFX indexMotor1 = new TalonFX(Constants.StorageConstants.indexMotorID);
  TalonFX indexMotor2 = new TalonFX(Constants.StorageConstants.indexMotor2ID);
  DigitalInput LimitSwitch = new DigitalInput(Constants.StorageConstants.limitid);
  public MotorAlignmentValue MotorAlignment = MotorAlignmentValue.Aligned; // Aligned or Opposed
  /** Creates a new Motor. */
  public Storage() {
    indexMotor2.setControl(new Follower(indexMotor1.getDeviceID(), MotorAlignment));
  }
  public void delayMotorStart(){
    if (getIndexer() == Indexer.ON) {
      timer.start();
      if (timer.hasElapsed(3)) {
        timer.stop();
        timer.reset();
        indexMotor1.setControl(new DutyCycleOut(0.6));
      } 
    } else {
      indexMotor1.setControl(new DutyCycleOut(0));
    }
    
  }
  public enum Phase {
    ATTACK,
    DEFENSE,
  }
  public enum Indexer {
    ON,
    OFF,
  }


public void moveMotor(double speed) {
   if (getIndexer() == Indexer.ON)
  { 
    flyWheel.setControl(new DutyCycleOut(speed));
    currentSpeed = speed;
  }
  else
  {
    flyWheel.setControl(new DutyCycleOut(0));
    currentSpeed = 0;
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

public boolean getRun() {
  return run;
}

public double getSpeed() {
  return currentSpeed;
}
public void fuckTheTimer(){
  timer.stop();
  timer.reset();
}

  @Override
  public void periodic() {

    delayMotorStart();


    // if("limit switch", LimitSwitch.get()) {
    //   detected = true;
    //   lastDetectionTime = timer.get();
    // }
    // else {
    //   detected = false;
    // }

    // if (timer.get() - lastDetectionTime >= 5.0) {
    //   moveMotor(0);
    // }
    SmartDashboard.putBoolean("limit switch", LimitSwitch.get());
    SmartDashboard.putBoolean("Detected", detected);
    SmartDashboard.putString("Phase", currentPhase.toString());
    SmartDashboard.putNumber("Speed", currentSpeed);
  }
}

