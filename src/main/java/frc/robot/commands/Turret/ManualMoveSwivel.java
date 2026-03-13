// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swivel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualMoveSwivel extends Command {
  boolean sign;
  /** Creates a new ManualMoveSwivel. */
  public ManualMoveSwivel(boolean sign) {
    addRequirements(RobotContainer.m_Swivel);
    this.sign = sign;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_Swivel.getState() == Swivel.States.INITIALIZED || RobotContainer.m_Swivel.getMode() == Swivel.Mode.MANUAL) {
    if(sign)
      RobotContainer.m_Swivel.moveSwivel(RobotContainer.m_Swivel.getSwivelPos() + 0.1);
    else
      RobotContainer.m_Swivel.moveSwivel(RobotContainer.m_Swivel.getSwivelPos() - 0.1);
    }
   }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Swivel.moveSwivel(RobotContainer.m_Swivel.getSwivelPos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
