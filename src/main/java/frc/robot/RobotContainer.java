// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.commands.AutoAim;

public class RobotContainer {
    private double MaxSpeed = Constants.kSpeedAt12VoltsMps; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Constants.MaxAngularRate; // Old: 0.05 rps (was extremely slow)
    private double AngularRate = MaxAngularRate;
    private double TurtleAngularRate = MaxAngularRate/3;       
    private double TurtleSpeed = MaxSpeed/3;

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
    private final JoystickButton zeroGyroJoystickButton = new JoystickButton(joystick, 10); // Button 5 for "Left Bumper"
    private final JoystickButton autoAim = new JoystickButton(joystick, 6);

    private static final int AXIS_X = 0; // X-axis (left/right)
    private static final int AXIS_Y = 1; // Y-axis (forward/backward)
    private static final int AXIS_TWIST = 2; // Twist (rotation)

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
    public final Vision m_vision = new Vision();
    
    // Auto-aim PID constants
    private final double kP = 0.03; // Proportional gain for rotation
    private final double txDeadband = 0; // Deadband in degrees (±0.75 degree)
    private final double minRotSpeed = 0.05; // Minimum rotation speed
    private final double maxRotSpeed = 1.5; // Maximum rotation speed
    
    // Auto-aim toggle state
    private boolean autoAimEnabled = false;

    // Slew rate limiters to prevent wheel slip and brownouts
    // Very high limits - only catches extreme changes to prevent slip on bad surfaces
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(15.0); // 15 m/s² - very fast
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(15.0); // 15 m/s² - very fast
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(20.0); // 20 rad/s² - very fast

     private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver")
      .add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2)
      .withPosition(0, 2);

    public RobotContainer() {
        // Configure the auto chooser AFTER drivetrain is initialized
        autoChooser.addOption("Straight Line", new PathPlannerAuto("Straight Line"));
        autoChooser.addOption("figure8", new PathPlannerAuto("figure 8"));


        // Add to Shuffleboard
        
        
        configureBindings();
    }
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    private void configureBindings() {
        // Button B toggles auto-aim on/off
        buttonB.onTrue(drivetrain.runOnce(() -> {
            autoAimEnabled = !autoAimEnabled;
            System.out.println("Auto-aim " + (autoAimEnabled ? "ENABLED" : "DISABLED"));
            SmartDashboard.putBoolean("Auto-Aim Enabled", autoAimEnabled);
        }));
        
        buttonA.onTrue(drivetrain.runOnce(() -> {
            MaxSpeed = TurtleSpeed;
            AngularRate = TurtleAngularRate;
            // Reset limiters to current value to prevent delay
            xLimiter.reset(0);
            yLimiter.reset(0);
            rotLimiter.reset(0);
        }));
        buttonA.onFalse(drivetrain.runOnce(() -> {
            MaxSpeed = Constants.kSpeedAt12VoltsMps;
            AngularRate = MaxAngularRate;
            // Reset limiters to current value to prevent delay
            xLimiter.reset(0);
            yLimiter.reset(0);
            rotLimiter.reset(0);
        }));
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double rotationSpeed;
                    
                    // Only use auto-aim if enabled AND we see an AprilTag
                    if (autoAimEnabled && m_vision.isVisible) {
                        double tx = m_vision.getTX();
                        
                        // Apply deadband - don't rotate if within ±1 degree
                        if (Math.abs(tx) > txDeadband) {
                            // Positive tx = target is to the right, rotate RIGHT (negative rotation)
                            // Negative tx = target is to the left, rotate LEFT (positive rotation)
                            rotationSpeed = -tx * kP;

                            // Apply minimum speed to overcome friction
                            if (Math.abs(rotationSpeed) > 0.01 && Math.abs(rotationSpeed) < minRotSpeed) {
                                rotationSpeed = Math.signum(rotationSpeed) * minRotSpeed;
                            }

                            // Limit maximum speed
                            rotationSpeed = Math.max(-maxRotSpeed, Math.min(maxRotSpeed, rotationSpeed));
                        } else {
                            rotationSpeed = 0; // Target centered, no rotation needed
                        }
                    } else {
                        // Auto-aim disabled or no target visible, use manual rotation from joystick
                        rotationSpeed = -applyDeadzone(joystick.getRawAxis(AXIS_TWIST), 0.2) * AngularRate;
                    }
                    
                    // Apply slew rate limiting to smooth acceleration and prevent wheel slip
                    double xVelocity = xLimiter.calculate(-applyDeadzone(joystick.getRawAxis(AXIS_Y), 0.2) * MaxSpeed);
                    double yVelocity = yLimiter.calculate(-applyDeadzone(joystick.getRawAxis(AXIS_X), 0.2) * MaxSpeed);
                    double rotVelocity = rotLimiter.calculate(rotationSpeed);
                    
                    return drive
                        .withVelocityX(xVelocity) // Drive forward with slew rate limiting
                        .withVelocityY(yVelocity) // Drive left with slew rate limiting
                        .withRotationalRate(rotVelocity); // Rotation with slew rate limiting
                }));
        
    

        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // buttonA.whileTrue(drivetrain.applyRequest(() -> brake));
        // buttonB.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(
        //         -applyDeadzone(joystick.getRawAxis(AXIS_Y), 0.2),
        //         -applyDeadzone(joystick.getRawAxis(AXIS_X), 0.2)))));

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
        zeroGyroJoystickButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}