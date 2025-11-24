// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.commands.AutoAim;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.05).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.3).withRotationalDeadband(MaxAngularRate * 0.3) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick joystick = new Joystick(0);
    private final JoystickButton buttonA = new JoystickButton(joystick, 1); // Button 1 for "A"
    private final JoystickButton buttonB = new JoystickButton(joystick, 2); // Button 2 for "B"
    private final JoystickButton buttonBack = new JoystickButton(joystick, 7); // Button 7 for "Back"
    private final JoystickButton buttonStart = new JoystickButton(joystick, 8); // Button 8 for "Start"
    private final JoystickButton buttonLeftBumper = new JoystickButton(joystick, 5); // Button 5 for "Left Bumper"
    private final JoystickButton autoAim = new JoystickButton(joystick, 6);

    private static final int AXIS_X = 0; // X-axis (left/right)
    private static final int AXIS_Y = 1; // Y-axis (forward/backward)
    private static final int AXIS_TWIST = 2; // Twist (rotation)

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision m_vision = new Vision();

    public RobotContainer() {
        configureBindings();
    }

    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    private void configureBindings() {
        autoAim.whileTrue(new AutoAim(drivetrain, m_vision));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-applyDeadzone(joystick.getRawAxis(AXIS_Y), 0.2) * MaxSpeed) // Drive forward
                                                                                                    // with deadzone
                        .withVelocityY(-applyDeadzone(joystick.getRawAxis(AXIS_X), 0.2) * MaxSpeed) // Drive left with
                                                                                                    // deadzone
                        .withRotationalRate(-applyDeadzone(joystick.getRawAxis(AXIS_TWIST), 0.2) * MaxAngularRate) // Rotate
                                                                                                                   // with
                                                                                                                   // deadzone
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        buttonA.whileTrue(drivetrain.applyRequest(() -> brake));
        buttonB.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(
                -applyDeadzone(joystick.getRawAxis(AXIS_Y), 0.2),
                -applyDeadzone(joystick.getRawAxis(AXIS_X), 0.2)))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        buttonBack.and(new JoystickButton(joystick, 4)) // Button 4 for "Y"
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        buttonBack.and(new JoystickButton(joystick, 3)) // Button 3 for "X"
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        buttonStart.and(new JoystickButton(joystick, 4)) // Button 4 for "Y"
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        buttonStart.and(new JoystickButton(joystick, 3)) // Button 3 for "X"
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        buttonLeftBumper.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}