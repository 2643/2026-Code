// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * AutoAimToPose: Automatically aims the turret at a target pose using robot odometry.
 * 
 * Usage:
 *   new AutoAimToPose(hubPose2d).withTimeout(2.0) // aim for 2 seconds
 *   new AutoAimToPose(hubPose2d) // aim indefinitely (until interrupted)
 */
public class AutoAimToPose extends Command {
  private final Pose2d targetPose;

  /**
   * Create a command that aims the turret at the given pose.
   * 
   * @param targetPose The Pose2d of the target (e.g., hub or speaker location)
   */
  public AutoAimToPose(Pose2d targetPose) {
    this.targetPose = targetPose;
    addRequirements(RobotContainer.m_Swivel);
  }

  @Override
  public void initialize() {
    // Optional: log start
    System.out.println("AutoAimToPose: Starting aim at pose (" 
        +this.targetPose.getX() + ", " + this.targetPose.getY() + ")");
  } 

  @Override
  public void execute() {
    // Calculate and command the turret angle to the target
    // RobotContainer.m_Swivel.turretTrackPose(this.targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("AutoAimToPose: Interrupted");
    } else {
      System.out.println("AutoAimToPose: Finished aiming");
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs indefinitely until interrupted or timeout.
    // If you want it to stop when the turret is close to target angle, add logic here:
    // return Math.abs(RobotContainer.m_Swivel.m_robotRelativeAngle - targetAngle) < tolerance;
    return false;
  }
}
