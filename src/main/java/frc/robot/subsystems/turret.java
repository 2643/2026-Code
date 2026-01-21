// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turret extends SubsystemBase {
  /** Creates a new turretx. */
  TalonFX motorY = new TalonFX(1);
  TalonFX motorX = new TalonFX(0);
  TalonFXConfiguration configs = new TalonFXConfiguration();
  public double target = 0;
  public double pos;
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
  public void position() {
      var motionmagicconfigs = configs.MotionMagic;
      var slot0configs = configs.Slot0;
  
      slot0configs.kP = 13;
      slot0configs.kI = 0;
      slot0configs.kD = 0;
  
      motionmagicconfigs.MotionMagicAcceleration = 20;
      motionmagicconfigs.MotionMagicCruiseVelocity = 20;
  
      motorY.getConfigurator().apply(configs);
      motorY.setPosition(0);
  }
  public void moveToPosY(double target){
    target = pos;
    motorY.setControl(motion.withPosition(target));
  }
  public void moveToPosX(double target){
    target = pos;
    motorX.setControl(motion.withPosition(target));
  }
  public void upMotorPosX(){
    moveToPosX(currentPosX() + 5);
  }
  public void downMotorPosX(){
    moveToPosX(currentPosX() - 5);
  }
  public void upMotorPosY(){
    moveToPosY(currentPosY() + 5);
  }
  public void downMotorPosY(){
    moveToPosY(currentPosY() - 5);
  }
  double currentPosY(){
    return motorY.getPosition().getValueAsDouble();
  }
  double currentPosX(){
    return motorX.getPosition().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
