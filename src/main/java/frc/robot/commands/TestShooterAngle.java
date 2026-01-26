// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * TEST COMMAND: Tests the shooter angle calculation using manual inputs from SmartDashboard.
 * This allows you to test the math without needing a real robot or Limelight.
 * 
 * To use:
 * 1. Run robot simulation
 * 2. Open Shuffleboard/Elastic/Glass
 * 3. Set "Turret/Test_Distance_M" to different distances
 * 4. Run this command (or it runs automatically)
 * 5. Watch "Turret/Test_Angle_Result" to see the calculated angle
 */
public class TestShooterAngle extends Command {
  private final Turret turret;

  public TestShooterAngle(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Test the angle calculation with values from SmartDashboard
    turret.testAngleCalculation();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // Runs continuously
  }
}
