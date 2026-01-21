// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Position extends SubsystemBase {
  double target = 0;
  public static TalonFX motor2 = new TalonFX(1); 
  
  TalonFXConfiguration configs = new TalonFXConfiguration();

  /** Creates a new Position. 

   public void upDownMotor() {
    if (target == 1) {
      target = 0;
      motor2.setControl(new MotionMagicVoltage(target));
    }
    else {
      target = 1;
      motor2.setControl(new MotionMagicVoltage(target));
    }
  }
  
 public Position() {
    var slot0config = configs.Slot0;
    var magicmotionconfig = configs.MotionMagic;
    configs.Slot0.kP = 5;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;

    magicmotionconfig.MotionMagicAcceleration = 20;
    magicmotionconfig.MotionMagicCruiseVelocity = 20;
    motor2.getConfigurator().apply(configs);
  }
    
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
*/