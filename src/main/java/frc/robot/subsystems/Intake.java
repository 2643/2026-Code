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

  TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeID);

  public Intake() {
  }

  public void moveIntake() {
    if (speed == 0) { 
      intakeMotor.setControl(new DutyCycleOut(Constants.IntakeConstants.intakeSpeed));
      speed = Constants.IntakeConstants.intakeSpeed;
    }
    else {
      intakeMotor.setControl(new DutyCycleOut(0));
      speed = 0;
    }
  }

  public boolean getSpeed() {
    if (speed == 0) { 
      return false;
    }
    else {
      return true;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target Intake Speed", speed);
    SmartDashboard.putNumber("Current Intake Speed", intakeMotor.getRotorVelocity().refresh().getValueAsDouble());
    SmartDashboard.putBoolean("Intaking", getSpeed());
  }
}

