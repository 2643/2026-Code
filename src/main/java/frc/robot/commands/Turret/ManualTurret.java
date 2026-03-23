// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swivel.Mode;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualTurret extends Command {
  boolean finish = false;
  /** Creates a new ManualTurret. */
  public ManualTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Swivel);
    addRequirements(RobotContainer.m_Hood);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(RobotContainer.m_Swivel.getMode() == Mode.MANUAL) {
      CommandScheduler.getInstance().schedule(new AutoAim());
      RobotContainer.m_Swivel.setMode(Mode.AUTOAIM);
    } else {
      RobotContainer.m_Swivel.setMode(Mode.MANUAL);
      RobotContainer.m_Swivel.moveSwivel(Constants.TurretConstants.manualSwivel);
      RobotContainer.m_Hood.moveHood(Constants.TurretConstants.manualHood);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
