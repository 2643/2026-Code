package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.TurretUtil;
import frc.robot.util.TurretUtil.TargetType;
import frc.robot.subsystems.Swivel.Mode;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.subsystems.Swivel.States;

public class AutoAim extends Command {
  private boolean isValid = false;
  private double turretAngle;

  
  private TargetType currentTarget = TargetType.HUB;

  public AutoAim() {
    SmartDashboard.putNumber("Turret Offset", Constants.TurretConstants.swivelOffset);
    addRequirements(RobotContainer.m_Swivel);
    addRequirements(RobotContainer.m_Hood);
  }

  @Override
  public void initialize() {
    System.out.println("AutoAimHub init");
    Constants.TurretConstants.running = true;
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
    double robotVelX = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
    double robotVelY = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;
    // TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, currentTarget);
    TurretUtil.ShotSolution solution = TurretUtil.computeLeadShotSolution(robotPose, robotVelX, robotVelY, currentTarget);

    isValid = solution.isValid;
    Constants.TurretConstants.swivelOffset = SmartDashboard.getNumber("Turret Offset", Constants.TurretConstants.swivelOffset);

    // System.out.println("AutoAimHub valid=" + isValid + " turretDeg=" + (solution.turretAngleDegrees) + " dist=" + solution.distanceMeters);
    SmartDashboard.putNumber("Turret Angle", solution.turretAngleDegrees);
    // SmartDashboard.putNumber("Shoot Turret Angle", movesolution.turretAngleDegrees);

    // SmartDashboard.putNumber("robotVelX",  robotVelX);
    // SmartDashboard.putNumber("robotVelY",  robotVelY);

    // SmartDashboard.putNumber("Right Angle", solution.turretAngleDegrees/Constants.TurretConstants.swivelGearRatio);
    // SmartDashboard.putNumber("Move Angle", movesolution.turretAngleDegrees/Constants.TurretConstants.swivelGearRatio);

    SmartDashboard.putNumber("Turret Dist", solution.distanceMeters);
    SmartDashboard.putBoolean("isValid", isValid);
    turretAngle = solution.turretAngleDegrees;

    // replace this with charlie hard limit mid thing
    // if (isValid) {
    // if(turretAngle <= Constants.TurretConstants.swivelHardLimit2)
    //   turretAngle = Constants.TurretConstants.swivelSoftLimit2;
    // if(turretAngle >= Constants.TurretConstants.swivelHardLimit1)
    //   turretAngle = Constants.TurretConstants.swivelSoftLimit1;

    turretAngle += Constants.TurretConstants.antiRotationOffset;
    RobotContainer.m_Swivel.moveSwivel(turretAngle);
    RobotContainer.m_Hood.moveHood(solution.trajectoryAngleDegrees-0);

    // if (RobotContainer.m_Storage.getWheel() == frc.robot.subsystems.Storage.Wheel.ON) {
    //   RobotContainer.m_Storage.moveWheel(solution.shooterSpeedRPS);
    //   // }
    // }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoAim ended, interrupted=" + interrupted);
    // Reschedule only if interrupted (by another command), not if naturally ending
    // if (interrupted) {
    //   CommandScheduler.getInstance().schedule(new AutoAim());
    // }
    Constants.TurretConstants.running = false;

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}