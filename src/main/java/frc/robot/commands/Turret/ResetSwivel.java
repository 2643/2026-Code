// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import frc.robot.subsystems.Swivel.States;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetSwivel extends Command {
  boolean finish = false;
  /** Creates a new InitTurret. */
  public ResetSwivel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Swivel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!RobotContainer.m_Swivel.getSwivelLimit()) {
      RobotContainer.m_Swivel.setState(States.NOT_INITIALIZED);
      finish = true;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_Swivel.getSwivelLimit()) {
      RobotContainer.m_Swivel.moveSwivel(RobotContainer.m_Swivel.getSwivelPos() + 0.05);
      finish = false;
    }
    else {
      finish = true;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!(RobotContainer.m_Swivel.getState() == States.NOT_INITIALIZED)) {
      RobotContainer.m_Swivel.startTimer();
      RobotContainer.m_Swivel.setSwivelPos(0); 
      RobotContainer.m_Swivel.setState(States.INITIALIZED);
      RobotContainer.m_Swivel.moveSwivel(Constants.TurretConstants.manualSwivel);
    } else {
      System.out.println("Swivel Disabled");
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
