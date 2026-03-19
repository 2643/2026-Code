// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.controller;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Storage.Toggle;
import frc.robot.util.Limelight4;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Storage.ToggleIndexer;
import frc.robot.commands.Storage.ToggleWheel;
import frc.robot.commands.Turret.ManualHoodDown;
import frc.robot.commands.Turret.ManualHoodUp;
import frc.robot.commands.Turret.ManualMoveSwivel;
import frc.robot.commands.Turret.ManualTurret;
import frc.robot.commands.Turret.ResetHood;
import frc.robot.commands.Turret.ResetSwivel;
import frc.robot.commands.Turret.Scram;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swivel;
import frc.robot.subsystems.Vision;



public class RobotContainer {
    // slow mode
    private final double kSlowMultiplier = 0.4;
    private final double normalMaxSpeed = Constants.OperatorConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double normalMaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity
    private double MaxSpeed = normalMaxSpeed;
    private double MaxAngularRate = normalMaxAngularRate;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.3).withRotationalDeadband(MaxAngularRate * 0.3)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
//     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//     private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static Joystick driver = new Joystick(Constants.driverPort);
    private final static Joystick operator = new Joystick(Constants.operatorPort);
    private final static Joystick progJoystick = new Joystick(Constants.progJoystickPort);

    public final static JoystickButton wheel = new JoystickButton(operator, Constants.StorageConstants.wheelPort);
    public final static JoystickButton indexer = new JoystickButton(operator, Constants.StorageConstants.indexerPort);
    public final static JoystickButton reverse = new JoystickButton(driver, 3);   
    public final static JoystickButton hootReinit = new JoystickButton(operator, Constants.resetGyroPort);
    

    public final Swerve drivetrain = Constants.OperatorConstants.createDrivetrain();
    // Use the actual Limelight network table name on your robot
    private final Limelight4 m_limelight = new Limelight4("limelight-allen");
    public final Vision m_Vision = new Vision();
    public final static Intake m_Intake = new Intake(); 
    public static final Hood m_Hood = new Hood();
    public static final Storage m_Storage = new Storage();
    public static final Swivel m_Swivel = new Swivel();

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver")
                .add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2)
                .withPosition(0, 2);

        // Trapezoidal limiter (fast ramp). Tune values as needed.
        // private final TrapezoidLimiter m_trapezoidLimiter = new TrapezoidLimiter(10.0, 20.0);

    
        public RobotContainer() {
            NamedCommands.registerCommand("Intake", new StartIntake(true));
            NamedCommands.registerCommand("Shoot", new ToggleWheel(Phase.ATTACK));
            NamedCommands.registerCommand("ManualTurret", new ManualTurret());
            // NamedCommands.registerCommand("Reset", new ResetTurret());
            NamedCommands.registerCommand("ResetHood", new ResetHood());
            NamedCommands.registerCommand("ResetSwivel", new ResetSwivel());
            NamedCommands.registerCommand("ToggleWheel", new ToggleWheel(m_Storage.getPhase()));
            NamedCommands.registerCommand("ToggleIndexer", new ToggleIndexer(true));




            configureBindings();

            //put autochooser options here
            autoChooser.addOption("S1 Shoot", new PathPlannerAuto("S1-O-Shoot"));
            autoChooser.addOption("S2 Shoot", new PathPlannerAuto("S2-MID-Shoot"));
            autoChooser.addOption("Straight Line", new PathPlannerAuto("Straight Line"));
            // autoChooser.addOption("Test", new PathPlannerAuto("rot"));
            autoChooser.addOption("null", null);
            autoChooser.addOption("S1-back", new PathPlannerAuto("Shoot-S1"));
            autoChooser.addOption("S2-back", new PathPlannerAuto("Shoot-S2"));
            autoChooser.addOption("S3-back", new PathPlannerAuto("Shoot-S3"));
            // Configure Limelight field preset to the 2026 rebuilt field by default.
            // This remaps incoming Limelight poses into the 2026 field coordinates.
            // If you need to tweak offsets, call m_limelight.setFieldTransform(xMeters, yMeters, rotDegrees).
            try {
                m_limelight.selectFieldPreset("2026-rebuilt");
                // Some Limelight configs have Y inverted relative to WPILib's
                // convention (X forward, Y left). If the pose moves opposite the
                // robot motion (e.g. moving right makes pose move left), flip Y.
                m_limelight.setFieldAxisFlip(false, true);
            } catch (Throwable t) {
                // ignore
            }
        }

    /**
     * Returns the last (x,y) from the Limelight if available (meters).
     */
    public Optional<double[]> getLimelightLastXY() {
        return m_limelight.getLastXY();
    }

    /** Returns the last Limelight Pose2d if available. */
    public Optional<Pose2d> getLimelightLastPose() {
        return m_limelight.getLastPose();
    }
    
        private double applyDeadzone(double value, double deadzone) {
            return Math.abs(value) > deadzone ? value : 0.0;
        }
    
        private void configureBindings() {
            toggle.onTrue(new Toggle());
            intake.onTrue(new StartIntake(true));
            wheel.onTrue(new ToggleWheel(m_Storage.getPhase()));
            indexer.onTrue(new ToggleIndexer(true));
            reverse.onTrue(new ToggleIndexer(false));
            reverse.onTrue(new StartIntake(false));
            manualTurret.onTrue(new ManualTurret());
            scram.onTrue(new Scram());
            hootReinit.onTrue(new ResetHood());
            swivelUp.whileTrue(new ManualMoveSwivel(true));
            swivelDown.whileTrue(new ManualMoveSwivel(false));
            hoodDown.onTrue(new ManualHoodDown());
            hoodUp.onTrue(new ManualHoodUp());
            
           
           
            PROGintake.onTrue(new StartIntake(true));
            PROGwheel.onTrue(new ToggleWheel(m_Storage.getPhase()));
            PROGindexer.onTrue(new ToggleIndexer(true));
            PROGreverse.onTrue(new ToggleIndexer(false));
            PROGreverse.onTrue(new StartIntake(false));
            PROGmanualTurret.onTrue(new ManualTurret());
            PROGhootReinit.onTrue(new ResetHood());
            PROGswivelUp.whileTrue(new ManualMoveSwivel(true));
            PROGswivelDown.whileTrue(new ManualMoveSwivel(false));

            // hoodUp.whileTrue(new ManualMoveHood(true));
            // hoodDown.whileTrue(new ManualMoveHood(false));

            



            // autoAim.whileTrue(new AutoAim(drivetrain, m_Vision));
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(  
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // raw desired velocities from joystick
                double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.2) * MaxSpeed; // forward
                double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.2) * MaxSpeed; // left
                double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.2) * MaxAngularRate; // rotate

                // double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.2) * MaxSpeed; // forward
                // double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.2) * MaxSpeed; // left
                // double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.2) * MaxAngularRate; // rotate

                // Apply trapezoidal limiter (fast ramp)
                // double[] smoothed = m_trapezoidLimiter.calculate(desiredX, desiredY, desiredOmega);
                if (PROGdesiredX != 0 || PROGdesiredY != 0 || PROGdesiredOmega != 0) {
                    return drive
                    .withVelocityX(PROGdesiredX)
                    .withVelocityY(PROGdesiredY)
                    .withRotationalRate(PROGdesiredOmega);
                } else {
                    return drive
                    .withVelocityX(desiredX)
                    .withVelocityY(desiredY)
                    .withRotationalRate(desiredOmega);}
            }));
    
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

            PROGzeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

            // slow mode (hold button 6 to reduce speeds)
            PROGslowMode.onTrue(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed * kSlowMultiplier;
                MaxAngularRate = normalMaxAngularRate * kSlowMultiplier;
            }));
            PROGslowMode.onFalse(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed;
                MaxAngularRate = normalMaxAngularRate;
            }));
    
            drivetrain.registerTelemetry(logger::telemeterize);
            // start Limelight updates (pushes initial pose immediately and then periodically)
            m_limelight.startUpdating(drivetrain, 0.2);
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
   return autoChooser.getSelected();
  }
}
