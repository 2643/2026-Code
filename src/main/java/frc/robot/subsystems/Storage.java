// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */

  
  TalonFX motor1 = new TalonFX(0);
  DigitalInput limitSwitch = new DigitalInput(1);
   public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  public void move(double Speed) {
    motor1.setControl(new DutyCycleOut(Speed));
  }
  public Storage() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
