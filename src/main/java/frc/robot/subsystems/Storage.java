// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;


public class Storage extends SubsystemBase {
  public double currentSpeed = 0;
  public boolean detected = false;
  public boolean run = false;
  private Timer timer = new Timer();
  private double lastDetectionTime = 0;
  Phase currentPhase = Phase.ATTACK;
  TalonFX motor = new TalonFX(Constants.StorageConstants.motorid);
  DigitalInput limitSwitch = new DigitalInput(Constants.StorageConstants.limitid);
  /** Creates a new Motor. */
  public Storage() {
    timer.start(); // Start the timer when subsystem is created
  }

  public enum Phase {
    ATTACK,
    DEFENSE,
  }


public void moveMotor(double speed) {
   if (currentSpeed == 0)
  { 
    motor.setControl(new DutyCycleOut(speed));
    currentSpeed = speed;
  }
  else
  {
    motor.setControl(new DutyCycleOut(0));
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

