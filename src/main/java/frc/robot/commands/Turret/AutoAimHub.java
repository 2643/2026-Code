package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TurretUtil;
import frc.robot.util.TurretUtil.TargetType;
import frc.robot.subsystems.Swivel.Mode;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.subsystems.Swivel.States;

public class AutoAimHub extends Command {
  private boolean isValid = false;
  private TargetType currentTarget = TargetType.HUB;

  public AutoAimHub() {
    addRequirements(RobotContainer.m_Swivel);
    addRequirements(RobotContainer.m_Hood);
    addRequirements(RobotContainer.m_Storage);
  }

  @Override
  public void initialize() {

}

  @Override
  public void execute() {
    if (RobotContainer.m_Swivel.getMode() != Mode.AUTOAIM) {
      return;
    }

    if (RobotContainer.m_Swivel.getState() != States.INITIALIZED) {
      return;
    }

    Phase currentPhase = RobotContainer.m_Storage.getPhase();
    
    if (currentPhase == Phase.ATTACK) {
      currentTarget = TargetType.HUB;
    } else if (currentPhase == Phase.DEFENSE) {
      var robotPose = RobotContainer.drivetrain.getState().Pose;
      currentTarget = TurretUtil.getNearestPassTargetType(robotPose);
    } else {
      return;
    }

    var robotPose = RobotContainer.drivetrain.getState().Pose;

    TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, currentTarget);

    isValid = solution.isValid;

    if (isValid) {
      RobotContainer.m_Swivel.moveSwivel(solution.turretAngleDegrees);
      RobotContainer.m_Hood.moveHood(solution.trajectoryAngleDegrees);
      
      if (RobotContainer.m_Storage.getWheel() == frc.robot.subsystems.Storage.Wheel.ON) {
        RobotContainer.m_Storage.moveWheel(solution.shooterSpeedRPS);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}