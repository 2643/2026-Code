// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {
  public double speed = 0;

  TalonFX motor = new TalonFX(Constants.IntakeConstants.intakeID);

  public Intake() {
  }

  public void moveMotor() {
    if (speed == 0) { 
      motor.setControl(new DutyCycleOut(Constants.IntakeConstants.intakeSpeed));
      speed = Constants.IntakeConstants.intakeSpeed;
    }
    else {
      motor.setControl(new DutyCycleOut(0));
      speed = 0;
    }
  }

  public boolean getSpeed() {
    if (speed == 0) { 
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", speed);
  }
}

