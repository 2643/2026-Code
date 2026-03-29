// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetPoseMidLeft extends Command {
  boolean finish = false;
  /** Creates a new ResetPose. */
  public ResetPoseMidLeft() {
    addRequirements(RobotContainer.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var driveState = RobotContainer.drivetrain.getState();
      Rotation2d headingDeg = driveState.Pose.getRotation();
    if(Constants.isRed) {
      RobotContainer.drivetrain.resetPose(new Pose2d(15, 2.25, headingDeg));
     } else {
      RobotContainer.drivetrain.resetPose(new Pose2d(2, 6, headingDeg));

     }
     finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
