// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.Optional;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.FuelVisionConstants;
// import frc.robot.RobotContainer;
// // import frc.robot.subsystems.FuelTracker;
// import frc.robot.subsystems.Swerve;

// /**
//  * Drives toward the nearest tracked fuel: turns to face it while closing distance,
//  * robot-centric, so the (front-mounted) intake ends up running it over.
//  *
//  * <p>Bind with {@code whileTrue} for driver use - it also ends on its own when no
//  * fuel has been tracked for {@link FuelVisionConstants#kFuelMemorySeconds}, so it
//  * is safe in autos too (e.g. as the "DriveToFuel" NamedCommand).
//  *
//  * <p>Speed scales with distance (P control), clamped to kChaseMaxSpeedMetersPerSec.
//  * Run the intake alongside this command to actually pick the fuel up.
//  */
// public class DriveToFuel extends Command {

//   private final Swerve drivetrain = RobotContainer.drivetrain;
//   private final FuelTracker tracker = RobotContainer.m_FuelTracker;

//   private final SwerveRequest.RobotCentric chaseRequest = new SwerveRequest.RobotCentric()
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//   private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

//   public DriveToFuel() {
//     // Only the drivetrain is a requirement; FuelTracker is read-only.
//     addRequirements(drivetrain);
//   }

//   @Override
//   public void execute() {
//     Optional<Translation2d> closest = tracker.getClosestFuelRobotRelative();
//     if (closest.isEmpty()) {
//       drivetrain.setControl(idle);
//       return;
//     }

//     Translation2d fuel = closest.get(); // +X forward, +Y left, meters
//     double distance = fuel.getNorm();
//     double bearing = Math.atan2(fuel.getY(), fuel.getX()); // rad, +left

//     double speed = MathUtil.clamp(
//         FuelVisionConstants.kChaseTranslationP * distance,
//         0.0,
//         FuelVisionConstants.kChaseMaxSpeedMetersPerSec);

//     double omega = MathUtil.clamp(
//         FuelVisionConstants.kChaseRotationP * bearing,
//         -FuelVisionConstants.kChaseMaxAngularRateRadPerSec,
//         FuelVisionConstants.kChaseMaxAngularRateRadPerSec);

//     // Drive straight at the fuel (robot-centric) while rotating to face it.
//     drivetrain.setControl(chaseRequest
//         .withVelocityX(speed * Math.cos(bearing))
//         .withVelocityY(speed * Math.sin(bearing))
//         .withRotationalRate(omega));
//   }

//   @Override
//   public boolean isFinished() {
//     // No fuel tracked anymore (intaked, or lost for longer than the memory window).
//     return !tracker.hasFuel();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.setControl(idle);
//   }
// }
