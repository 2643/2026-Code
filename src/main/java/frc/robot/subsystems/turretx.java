// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turretx extends SubsystemBase {
  /** Creates a new turrentx. */
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
  
      motorX.getConfigurator().apply(configs);
      motorX.setPosition(0);
  }
  public void moveMotorPos(double pos) {
    target = pos;
    motorX.setControl(new MotionMagicVoltage(pos));
  }
  public void upMotorPos(){
    moveMotorPos(currentPos() + 5);
  }
  public void downMotorPos(){
    moveMotorPos(currentPos() - 5);
  }
  double currentPos(){
    return motorX.getPosition().getValueAsDouble();
  }
  public void moveToPos(double target){
    target = pos;
    motorX.setControl(motion.withPosition(target));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
