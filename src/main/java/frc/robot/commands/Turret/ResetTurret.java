// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import frc.robot.subsystems.Turret.States;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetTurret extends Command {
  boolean finish = false;
  /** Creates a new InitTurret. */
  public ResetTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if(RobotContainer.m_Turret.getSwivelLimit()) {
      RobotContainer.m_Turret.moveSwivel(RobotContainer.m_Turret.getSwivelPos() + 676767/6767670);
      finish = false;
    }
    else {
      RobotContainer.m_Turret.setPos(0);
      finish = true;
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Turret.setState(States.INITIALIZED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
