// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * TEST COMMAND: Tests the angle-to-motor-position conversion.
 * Useful for calibrating your gearing constants.
 * 
 * To use:
 * 1. Set "Turret/Test_Set_Angle_Deg" in SmartDashboard (e.g., 30 degrees)
 * 2. Run this command
 * 3. Observe the motor position in "Turret/Hood_Motor_Position"
 * 4. Verify on physical robot that hood moves to correct angle
 * 5. Adjust MOTOR_ROTATIONS_PER_DEGREE in Constants if needed
 */
public class TestHoodGearing extends Command {
  private final Turret turret;

  public TestHoodGearing(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Get test angle from SmartDashboard
    double testAngle = SmartDashboard.getNumber("Turret/Test_Set_Angle_Deg", 30.0);
    
    // Set the hood to that angle
    turret.setHoodAngle(testAngle);
    
    // Display conversion info
    SmartDashboard.putString("HoodTest/Status", "Testing angle: " + testAngle + "°");
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("HoodTest/Status", "Test stopped");
  }

  @Override
  public boolean isFinished() {
    return false; // Runs continuously
  }
}
