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
// import frc.robot.Constants.ControllerConstants; (unused)
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Storage.Toggle;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Storage.ToggleIndexer;
import frc.robot.commands.Storage.ToggleWheel;
import frc.robot.commands.Turret.AutoAim;
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
    private final double kSlowMultiplier = 0.3;
    private final double normalMaxSpeed = Constants.OperatorConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double normalMaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity
    private double MaxSpeed = normalMaxSpeed;
    private double MaxAngularRate = normalMaxAngularRate;
    // Trapezoidal limiter to smooth joystick requests
    private final frc.robot.util.TrapezoidLimiter m_trapezoidLimiter = new frc.robot.util.TrapezoidLimiter(3.0, 3.0, 6.0);
    // Shuffleboard tunables
    private final edu.wpi.first.networktables.GenericEntry m_rateXEntry;
    private final edu.wpi.first.networktables.GenericEntry m_rateYEntry;
    private final edu.wpi.first.networktables.GenericEntry m_rateOmegaEntry;

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

    public final static JoystickButton wheel = new JoystickButton(operator, Constants.ControllerConstants.square);
    public final static JoystickButton indexer = new JoystickButton(operator, Constants.ControllerConstants.x);
    public final static JoystickButton reverse = new JoystickButton(driver, Constants.ControllerConstants.circle);   
    public final static JoystickButton hootReinit = new JoystickButton(operator, Constants.ControllerConstants.rightNiche);
    
    public static final JoystickButton intake = new JoystickButton(driver, Constants.ControllerConstants.x);
    public static final JoystickButton manualTurret = new JoystickButton(operator, Constants.ControllerConstants.triangle);
    public final static JoystickButton toggle = new JoystickButton(operator, Constants.ControllerConstants.circle);
    public final static JoystickButton zeroGyro = new JoystickButton(driver, Constants.ControllerConstants.rightNiche);
    public final static JoystickButton slowMode = new JoystickButton(driver, Constants.ControllerConstants.RB);
    public final static JoystickButton hoodDown = new JoystickButton(operator, Constants.ControllerConstants.LB);
    public final static JoystickButton hoodUp = new JoystickButton(operator, Constants.ControllerConstants.RB);
    public final static JoystickButton scram = new JoystickButton(operator, Constants.ControllerConstants.leftNiche);
    public final static JoystickButton swivelUp = new JoystickButton(operator, Constants.ControllerConstants.ZL);
    public final static JoystickButton swivelDown = new JoystickButton(operator, Constants.ControllerConstants.ZR);

    public static final JoystickButton PROGintake = new JoystickButton(progJoystick, Constants.ControllerConstants.x);
    public static final JoystickButton PROGmanualTurret = new JoystickButton(progJoystick, Constants.ControllerConstants.triangle);
    // public final static JoystickButton PROGtoggle = new JoystickButton(progJoystick, Constants.ControllerConstants.circle);
    public final static JoystickButton PROGzeroGyro = new JoystickButton(progJoystick, Constants.ControllerConstants.rightNiche);
    public final static JoystickButton PROGslowMode = new JoystickButton(progJoystick, Constants.ControllerConstants.RB);
    // public final static JoystickButton PROGhoodDown = new JoystickButton(progJoystick, Constants.ControllerConstants.LB);
    // public final static JoystickButton PROGhoodUp = new JoystickButton(progJoystick, Constants.ControllerConstants.RB);
    // public final static JoystickButton PROGscram = new JoystickButton(progJoystick, Constants.ControllerConstants.leftNiche);
    public final static JoystickButton PROGswivelUp = new JoystickButton(progJoystick, Constants.ControllerConstants.ZL);
    public final static JoystickButton PROGswivelDown = new JoystickButton(progJoystick, Constants.ControllerConstants.ZR);
    public final static JoystickButton PROGwheel = new JoystickButton(progJoystick, Constants.ControllerConstants.LB);
    public final static JoystickButton PROGindexer = new JoystickButton(progJoystick, Constants.ControllerConstants.square);
    public final static JoystickButton PROGreverse = new JoystickButton(progJoystick, Constants.ControllerConstants.circle);   
    public final static JoystickButton PROGhootReinit = new JoystickButton(progJoystick, Constants.ControllerConstants.leftNiche);
    
    
    public final static Swerve drivetrain = Constants.OperatorConstants.createDrivetrain();
    public final Vision m_Vision = new Vision();
    public final static Intake m_Intake = new Intake(); 
    public static final Hood m_Hood = new Hood();
    public static final Storage m_Storage = new Storage();
    public static final Swivel m_Swivel = new Swivel();

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver")
                .add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2)
                .withPosition(0, 2);
    
        public RobotContainer() {
            NamedCommands.registerCommand("Intake", new ToggleIntake(true));
            NamedCommands.registerCommand("Shoot", new ToggleWheel(Phase.ATTACK));
            NamedCommands.registerCommand("ManualTurret", new ManualTurret());
            NamedCommands.registerCommand("AutoAim", new AutoAim());
            NamedCommands.registerCommand("Zero Gyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            NamedCommands.registerCommand("ResetHood", new ResetHood());
            NamedCommands.registerCommand("ResetSwivel", new ResetSwivel());
            NamedCommands.registerCommand("ToggleWheel", new ToggleWheel(m_Storage.getPhase()));
            NamedCommands.registerCommand("ToggleIndexer", new ToggleIndexer(true));
            configureBindings();

            autoChooser.addOption("S1 O Shoot", new PathPlannerAuto("S1-O-Shoot"));
            autoChooser.addOption("S3 Mid Shoot", new PathPlannerAuto("S3-MID-Shoot"));
            autoChooser.addOption("Straight Line", new PathPlannerAuto("Straight Line"));
            autoChooser.addOption("null", null);
            autoChooser.addOption("S1 Shoot", new PathPlannerAuto("Shoot-S1"));
            autoChooser.addOption("S2 Shoot", new PathPlannerAuto("Shoot-S2"));
            autoChooser.addOption("S3 Shoot", new PathPlannerAuto("Shoot-S3"));
            // Configure Limelight field preset to the 2026 rebuilt field by default.
            // This remaps incoming Limelight poses into the 2026 field coordinates.
            // If you need to tweak offsets, call m_limelight.setFieldTransform(xMeters, yMeters, rotDegrees).
            try {
                SmartDashboard.putString("Limelight/Mode", "RobotPeriodic-MegaTag2");
            } catch (Throwable t) {
                // ignore
            }
            // Create Shuffleboard tunables for the trapezoid limiter (Driver tab)
            var tab = Shuffleboard.getTab("Driver");
            m_rateXEntry = tab.add("Limiter Rate X (m/s/s)", 2.0).withPosition(4, 2).withSize(2, 1).getEntry();
            m_rateYEntry = tab.add("Limiter Rate Y (m/s/s)", 2.0).withPosition(6, 2).withSize(2, 1).getEntry();
            m_rateOmegaEntry = tab.add("Limiter Rate Omega (rad/s/s)", 4.0).withPosition(8, 2).withSize(2, 1).getEntry();
        }

    /**
     * Returns the last (x,y) from the Limelight if available (meters).
     */
    public Optional<double[]> getLimelightLastXY() {
        return Optional.empty();
    }

    /** Returns the last Limelight Pose2d if available. */
    public Optional<Pose2d> getLimelightLastPose() {
        return Optional.empty();
    }
    
        private double applyDeadzone(double value, double deadzone) {
            return Math.abs(value) > deadzone ? value : 0.0;
        }
    
        private void configureBindings() {
            toggle.onTrue(new Toggle());
            intake.onTrue(new ToggleIntake(true));
            wheel.onTrue(new ToggleWheel(m_Storage.getPhase()));
            indexer.onTrue(new ToggleIndexer(true));
            reverse.onTrue(new ToggleIndexer(false));
            reverse.onTrue(new ToggleIntake(false));
            manualTurret.onTrue(new ManualTurret());
            scram.onTrue(new Scram());
            hootReinit.onTrue(new ResetHood());
            swivelUp.whileTrue(new ManualMoveSwivel(true));
            swivelDown.whileTrue(new ManualMoveSwivel(false));
            hoodDown.onTrue(new ManualHoodDown());
            hoodUp.onTrue(new ManualHoodUp());
            
           
           
            PROGintake.onTrue(new ToggleIntake(true));
            PROGwheel.onTrue(new ToggleWheel(m_Storage.getPhase()));
            PROGindexer.onTrue(new ToggleIndexer(true));
            PROGreverse.onTrue(new ToggleIndexer(false));
            PROGreverse.onTrue(new ToggleIntake(false));
            PROGmanualTurret.onTrue(new ManualTurret());
            // PROGmanualTurret.onTrue(new AutoAim());
            PROGhootReinit.onTrue(new ResetHood());
            PROGswivelUp.whileTrue(new ManualMoveSwivel(true));
            PROGswivelDown.whileTrue(new ManualMoveSwivel(false));
            
        drivetrain.setDefaultCommand(  
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // raw desired velocities from joystick
                double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.1) * MaxSpeed; // forward
                double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.1) * MaxSpeed; // left
                double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.1) * MaxAngularRate; // rotate

                double PROGdesiredX = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_Y), 0.1) * MaxSpeed; // forward
                double PROGdesiredY = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_X), 0.1) * MaxSpeed; // left
                double PROGdesiredOmega = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_TWIST), 0.1) * MaxAngularRate; // rotate

                // double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.2) * MaxSpeed; // forward
                // double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.2) * MaxSpeed; // left
                // double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.2) * MaxAngularRate; // rotate

                // Read Shuffleboard tunables and apply them to the limiter
                try {
                    double rx = m_rateXEntry.getDouble(3.0);
                    double ry = m_rateYEntry.getDouble(3.0);
                    double ro = m_rateOmegaEntry.getDouble(6.0);
                    m_trapezoidLimiter.setRates(rx, ry, ro);
                } catch (Throwable t) {
                    // ignore and use current rates
                }
                // Apply trapezoidal limiter (slightly steep ramp)
                double[] smoothed = m_trapezoidLimiter.calculate(desiredX, desiredY, desiredOmega);
                if (PROGdesiredX != 0 || PROGdesiredY != 0 || PROGdesiredOmega != 0) {
                    Constants.TurretConstants.antiRotationOffset = Constants.TurretConstants.antiMultiplier * PROGdesiredOmega;
                    SmartDashboard.putNumber("antiRotation", Constants.TurretConstants.antiRotationOffset);
                    return drive
                    .withVelocityX(PROGdesiredX)
                    .withVelocityY(PROGdesiredY)
                    .withRotationalRate(PROGdesiredOmega);
                } else {
                    Constants.TurretConstants.antiRotationOffset = Constants.TurretConstants.antiMultiplier * desiredOmega;
                    SmartDashboard.putNumber("antiRotation", Constants.TurretConstants.antiRotationOffset);
                    return drive
                    .withVelocityX(smoothed[0])
                    .withVelocityY(smoothed[1])
                    .withRotationalRate(smoothed[2]);}
            }));
    
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                    drivetrain.applyRequest(() -> idle).ignoringDisable(true));
          
            zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
