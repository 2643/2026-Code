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
  private double lastDetectionTime = 0;
  Phase currentPhase = Phase.ATTACK;
  TalonFX flyWheel = new TalonFX(Constants.StorageConstants.flyWheel);
  TalonFX storage1 = new TalonFX(Constants.StorageConstants.motorid1);
  TalonFX storage2 = new TalonFX(Constants.StorageConstants.motorid2);
  DigitalInput limitSwitch = new DigitalInput(Constants.StorageConstants.limitid);
  public MotorAlignmentValue MotorAlignment = MotorAlignmentValue.Opposed; // Aligned or Opposed
  /** Creates a new Motor. */
  public Storage() {
    storage1.setControl(new Follower(storage2.getDeviceID(), MotorAlignment));
    timer.start(); // Start the timer when subsystem is created
  }
  public void delayMotorStart(){
    if (timer.get() == 3) {
      storage1.setControl(new DutyCycleOut(0.6));
    }
    else {
      storage1.setControl(new DutyCycleOut(0));
    }
  }
  public enum Phase {
    ATTACK,
    DEFENSE,
  }


public void moveMotor(double speed) {
   if (currentSpeed == 0)
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

public boolean getRun() {
  return run;
}

public double getSpeed() {
  return currentSpeed;
}

  @Override
  public void periodic() {
    if(limitSwitch.get()) {
      detected = true;
      lastDetectionTime = timer.get();
    }
    else {
      detected = false;
    }

    if (timer.get() - lastDetectionTime >= 5.0) {
      moveMotor(0);
    }

    SmartDashboard.putBoolean("Detected", detected);
    SmartDashboard.putString("Phase", currentPhase.toString());
    SmartDashboard.putNumber("Speed", currentSpeed);
  }
}

