// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */

  
  public TalonFX storage = new TalonFX(0);
  public DigitalInput limitSwitch = new DigitalInput(1);
  // public boolean getLimitSwitch(){
  //   return limitSwitch.get();
  // }

  public void SetMotorSpeed(double Speed) {
    storage.setControl(new DutyCycleOut(Speed));
  }
  public Storage() {}

  public double getPosition() {
    return storage.getPosition().getValueAsDouble();
  }

  States state = States.ATTACK;
  public enum States {
    ATTACK,
    DEFENSE,

  }

  private final Timer timer = new Timer();
  private boolean isTimerRunning = false;
  private boolean isMotorRunning = false;

  public void toggleMotor() {
    if (isMotorRunning) {
      SetMotorSpeed(0);
    } else {
      SetMotorSpeed(0.75);
    }
    isMotorRunning = !isMotorRunning;
  }

  @Override
  public void periodic() {
    if (getLimitValue()) {
      if (!isTimerRunning) {
        timer.reset();
        timer.start();
        isTimerRunning = true;
      }
      SetMotorSpeed(0.75);
    } else if (isTimerRunning && timer.get() < 2.0) {
      SetMotorSpeed(0.75);
    } else {
      SetMotorSpeed(0);
      timer.stop();
      isTimerRunning = false;
    }

    switch (state) {
      case ATTACK:
        break;
      case DEFENSE:
        break;
    }
  }

  
  
  public boolean getLimitValue(){
    return limitSwitch.get();
  }
  
}

