// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Turret.ManualHoodDown;
import frc.robot.commands.Turret.ManualHoodUp;
import frc.robot.commands.Turret.ManualTurret;
import frc.robot.commands.Turret.SetEncoder;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Storage.Shoot;
import frc.robot.commands.Storage.Toggle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;


public class RobotContainer {
    // Normal and dynamic (current) max speed/rotation values. Slow mode multiplies these.
    private final double kSlowMultiplier = 0.4;
    private final double normalMaxSpeed = Constants.OperatorConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double normalMaxAngularRate = RotationsPerSecond.of(0.05).in(RadiansPerSecond); // max angular velocity
    private double MaxSpeed = normalMaxSpeed;
    private double MaxAngularRate = normalMaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.3).withRotationalDeadband(MaxAngularRate * 0.3)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
//     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//     private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static Joystick joystick = new Joystick(0);
    public static final JoystickButton start = new JoystickButton(joystick, Constants.IntakeConstants.intakePort);
    public static final JoystickButton manual_turret = new JoystickButton(joystick, Constants.TurretConstants.turretPort);
    public final static JoystickButton shoot = new JoystickButton(joystick, Constants.TurretConstants.shootPort);
    public final static JoystickButton toggle = new JoystickButton(joystick, Constants.StorageConstants.togglePort);
    // Button 10: zero the gyro / seed field-centric heading
    public final static JoystickButton zeroGyro = new JoystickButton(joystick, 10);
    // Button 6: hold for slow mode (reduced translation & rotation)
    public final static JoystickButton slowMode = new JoystickButton(joystick, 6);
    public final static JoystickButton hoodDown = new JoystickButton(joystick, 7);
    public final static JoystickButton hoodUp = new JoystickButton(joystick, 8);

    
        private static final int AXIS_X = 0; // X-axis 
        private static final int AXIS_Y = 1; // Y-axis 
        private static final int AXIS_TWIST = 2; // rotation
    
        public final Swerve drivetrain = Constants.OperatorConstants.createDrivetrain();
        public final Vision m_vision = new Vision();
        public final static Intake m_intake = new Intake();
        public static final Turret m_turret = new Turret();
        public static final Storage m_Storage = new Storage();
    
        public RobotContainer() {
            configureBindings();
        }
    
        private double applyDeadzone(double value, double deadzone) {
            return Math.abs(value) > deadzone ? value : 0.0;
        }
    
        private void configureBindings() {
            toggle.onTrue(new Toggle());
            start.onTrue(new StartIntake());
            shoot.onTrue(new Shoot(m_Storage.getPhase()));
            hoodDown.onTrue(new ManualHoodDown());
            hoodUp.onTrue(new ManualHoodUp());
            manual_turret.onTrue(new ManualTurret());


            // autoAim.whileTrue(new AutoAim(drivetrain, m_vision));
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
          

            //swerve code that came with the template
            // buttonA.whileTrue(drivetrain.applyRequest(() -> brake));
            // buttonB.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(
                    // -applyDeadzone(joystick.getRawAxis(AXIS_Y), 0.2),
                    // -applyDeadzone(joystick.getRawAxis(AXIS_X), 0.2)))));
    
            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            // buttonBack.and(new JoystickButton(joystick, 4)) // Button 4 for "Y"
            //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            // buttonBack.and(new JoystickButton(joystick, 3)) // Button 3 for "X"
            //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            // buttonStart.and(new JoystickButton(joystick, 4)) // Button 4 for "Y"
            //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            // buttonStart.and(new JoystickButton(joystick, 3)) // Button 3 for "X"
            //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
            // reset the field-centric heading on left bumper press
            // buttonLeftBumper.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            // zero gyro on button 10
            zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

            // slow mode (hold button 6 to reduce speeds)
            slowMode.onTrue(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed * kSlowMultiplier;
                MaxAngularRate = normalMaxAngularRate * kSlowMultiplier;
            }));
            slowMode.onFalse(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed;
                MaxAngularRate = normalMaxAngularRate;
            }));
    
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    
    
    
    
    
    /**
     * This class is where the bulk of the robot should be declared. Since Command-based is a
     * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
     * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
     * subsystems, commands, and trigger mappings) should be declared here.
     */
      // The robot's subsystems and commands are defined here...
      //public static final JoystickButton raise = new JoystickButton(controller, 2);
     

  /** The container for the robot. Contains subsystems, OI devices, and commands. */


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   return null;
  }
}
