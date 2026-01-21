// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public double speed = 0;
  TalonFX motor = new TalonFX(0);
  /** Creates a new Motor. */
  public Intake() {
    
  }

public void moveMotor() {
  System.out.println(speed);
  if (speed == 0)
  { 
    motor.setControl(new DutyCycleOut(0.6));
    speed = 0.6;
  }
  else
  {
    motor.setControl(new DutyCycleOut(0));
    speed = 0;
  }
}
public double getSpeed() {
  return speed;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

