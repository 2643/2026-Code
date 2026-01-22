// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretXReset extends Command {
  /** Creates a new TurretXReset. */
  public TurretXReset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_turret);
  }
  public boolean finish = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.m_turret.getLimitX()) {
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_turret.getLimitX()) {
      RobotContainer.m_turret.moveToPosX(RobotContainer.m_turret.currentPosX()-0.03);
    }
    if (!RobotContainer.m_turret.getLimitX()) {
      finish = true;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_turret.moveToPosX(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
