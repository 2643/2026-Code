package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    System.out.println("AutoAimHub init");
  }

  @Override
  public void execute() {
    boolean run = (RobotContainer.m_Swivel.getMode() == Mode.AUTOAIM)
        && (RobotContainer.m_Swivel.getState() == States.INITIALIZED);

    if (!run) {
      return;
    }

    Phase currentPhase = RobotContainer.m_Storage.getPhase();

    if (currentPhase == Phase.ATTACK) {
      currentTarget = TargetType.HUB;
    } else if (currentPhase == Phase.DEFENSE) {
      var robotPose = RobotContainer.drivetrain.getState().Pose;
      currentTarget = TurretUtil.getNearestPassTargetType(robotPose);
    }

    var robotPose = RobotContainer.drivetrain.getState().Pose;
    TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, currentTarget);

    isValid = solution.isValid;
    System.out.println("AutoAimHub valid=" + isValid + " turretDeg=" + (solution.turretAngleDegrees) + " dist=" + solution.distanceMeters);
    SmartDashboard.putNumber("Turret Angle", solution.turretAngleDegrees);
    SmartDashboard.putNumber("Turret DIst", solution.distanceMeters);

    SmartDashboard.putBoolean("isValid", isValid);

    // if (isValid) {
      RobotContainer.m_Swivel.moveSwivel(solution.turretAngleDegrees);
      RobotContainer.m_Hood.moveHood(solution.trajectoryAngleDegrees);

      if (RobotContainer.m_Storage.getWheel() == frc.robot.subsystems.Storage.Wheel.ON) {
        RobotContainer.m_Storage.moveWheel(solution.shooterSpeedRPS);
      // }
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoAimHub end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}