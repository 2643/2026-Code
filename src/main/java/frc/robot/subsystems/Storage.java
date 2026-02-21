// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */

  
  TalonFX motor1 = new TalonFX(Constants.StorageConstants.motorid);
  DigitalInput limitSwitch = new DigitalInput(Constants.StorageConstants.limitid);
  // public boolean getLimitSwitch(){
  //   return limitSwitch.get();
  // }

  public void SetMotorPosition(double Position) {
    motor1.setControl(new MotionMagicVoltage(Position));
  }
  public Storage() {}

  public double getPosition() {
    return motor1.getPosition().getValueAsDouble();
  }

  States state = States.ATTACK;
  public enum States {
    ATTACK,
    DEFENSE,
  }

  @Override
  public void periodic() {
    if (limitSwitch.get()) {
      SetMotorPosition(getPosition()+0.1);
    }
    
  switch (state) {
      case ATTACK:
        break;
      case DEFENSE:
        break;
    }
    // This method will be called once per scheduler run
  }
}
