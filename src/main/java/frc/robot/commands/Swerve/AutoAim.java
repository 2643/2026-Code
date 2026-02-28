package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAim extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final SwerveRequest.FieldCentric autoAimRequest;

  private final double kP = 0.03; // Proportional gain for rotation
  private final double minSpeed = 0.05; // Minimum rotation speed
  private final double maxSpeed = 1.5; // Maximum rotation speed

  public AutoAim(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    autoAimRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withRotationalDeadband(0.1);

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    System.out.println("hi");
  }

  @Override
  public void execute() {
    double rotationSpeed = 0;

    if (vision.isVisible) {
      // Calculate rotation speed based on yaw error
      rotationSpeed = -vision.yaw * kP;

      // Apply minimum speed to overcome friction
      if (Math.abs(rotationSpeed) > 0.01 && Math.abs(rotationSpeed) < minSpeed) {
        rotationSpeed = Math.signum(rotationSpeed) * minSpeed;
      }

      // Limit maximum speed
      rotationSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, rotationSpeed));
    }

    drivetrain.setControl(autoAimRequest
        .withVelocityX(0) // No forward/backward movement
        .withVelocityY(0) // No left/right movement
        .withRotationalRate(rotationSpeed));
  }

  @Override
  public boolean isFinished() {
    return vision.isVisible;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}