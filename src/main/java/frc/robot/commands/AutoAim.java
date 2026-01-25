package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAim extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final SwerveRequest.FieldCentric autoAimRequest;

  private final double kP = 0.03; // Proportional gain for rotation
  private final double txDeadband = 1.0; // Deadband in degrees (±1 degree)
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
    System.out.println("AutoAim started");
    vision.setAutoAimActive(true);
  }

  @Override
  public void execute() {
    double rotationSpeed = 0;


    if (vision.isVisible) {
      double tx = vision.getTX(); // Get horizontal offset
      
      // Apply deadband - don't rotate if within ±1 degree
      if (Math.abs(tx) > txDeadband) {
        // Positive tx = target is to the right, rotate RIGHT (negative rotation)
        // Negative tx = target is to the left, rotate LEFT (positive rotation)
        rotationSpeed = -tx * kP;

        // Apply minimum speed to overcome friction
        if (Math.abs(rotationSpeed) > 0.01 && Math.abs(rotationSpeed) < minSpeed) {
          rotationSpeed = Math.signum(rotationSpeed) * minSpeed;
        }

        // Limit maximum speed
        rotationSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, rotationSpeed));
      }
    }

    drivetrain.setControl(autoAimRequest
        .withVelocityX(0) // No forward/backward movement
        .withVelocityY(0) // No left/right movement
        .withRotationalRate(rotationSpeed));

   
  }

  @Override
  public boolean isFinished() {
    // Never finish automatically - only end when button is pressed to toggle off
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
    vision.setAutoAimActive(false);
    System.out.println("AutoAim ended");
  }
}