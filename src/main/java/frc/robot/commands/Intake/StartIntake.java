// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StartIntake extends Command {
  boolean finish;
  boolean sign;
  /** Creates a new start_intake. */
  public StartIntake(boolean sign) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Intake);
    this.sign = sign;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(sign)
      RobotContainer.m_Intake.moveIntake(Constants.IntakeConstants.intakeSpeed);
    else
      RobotContainer.m_Intake.moveIntake(-Constants.IntakeConstants.intakeSpeed);

    finish = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake", RobotContainer.m_Intake.getSpeed());
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
