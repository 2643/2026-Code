// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ResetPoseMidRight extends Command {
  boolean finish = false;

  public ResetPoseMidRight() {
    addRequirements(RobotContainer.drivetrain);
  }

  @Override
  public void initialize() {
    var driveState = RobotContainer.drivetrain.getState();
    Rotation2d headingDeg = driveState.Pose.getRotation();

    if (Constants.isRed) {
      RobotContainer.drivetrain.resetPose(new Pose2d(15, 5.75, headingDeg));
    } else {
      RobotContainer.drivetrain.resetPose(new Pose2d(2, 2, headingDeg));
    }

    finish = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finish;
  }
}