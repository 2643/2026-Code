// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * Command to automatically set the shooter angle based on Limelight AprilTag detection.
 */
public class AutoAimShooter extends Command {
  private final Turret turret;

  /**
   * Creates a new AutoAimShooter command.
   * 
   * @param turret The turret subsystem to use
   */
  public AutoAimShooter(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Continuously update shooter angle based on Limelight data
    turret.setShooterAngleFromLimelight();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // This command runs continuously until interrupted
    return false;
  }
}
