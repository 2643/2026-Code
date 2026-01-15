// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class gate extends SubsystemBase {
  /** Creates a new gate. */
  TalonFX gate = new TalonFX(3);
  double speed;
  double pos;
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  public void cycleGate(double speed) {
    gate.setControl(new DutyCycleOut(speed));
  }
  public void closeGate(double target){
    target = pos;
    gate.setControl(motion.withPosition(target));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
