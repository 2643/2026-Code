// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Storage.Indexer;
import frc.robot.subsystems.Storage.Phase;


 
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  boolean finish;
  Phase phase;

  public Shoot(Phase phase) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Storage);
    this.phase = phase;
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
    RobotContainer.m_Storage.fuckTheTimer();
    switch (RobotContainer.m_Storage.getIndexer()){
      case ON -> RobotContainer.m_Storage.setIndexer(Indexer.OFF);
      case OFF -> RobotContainer.m_Storage.setIndexer(Indexer.ON);
    }

    switch (phase) {
      case ATTACK -> RobotContainer.m_Storage.moveMotor(Constants.StorageConstants.attackSpeed);
      case DEFENSE -> RobotContainer.m_Storage.moveMotor(Constants.StorageConstants.defenseSpeed);
      default -> throw new AssertionError(phase.name());
    }
    
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
