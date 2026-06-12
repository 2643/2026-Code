// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TurretUtil;


public class Storage extends SubsystemBase {

  public enum Phase {
    ATTACK,
    DEFENSE,
  }

  public enum Indexer {
    ON,
    OFF,
  }

  public enum Wheel {
    ON,
    OFF,
  }

  public double wheelSpeed;
  public double targetWheelSpeed;
  public double targetIndexSpeed;
  public double indexSpeed;
  public boolean shoot = false;
  public boolean spin = false;


  TalonFX flyWheel = new TalonFX(Constants.StorageConstants.wheelID);
  TalonFX indexMotor1 = new TalonFX(Constants.StorageConstants.indexMotorID);
  TalonFX indexMotor2 = new TalonFX(Constants.StorageConstants.indexMotor2ID);

  DigitalInput indexLimit = new DigitalInput(Constants.StorageConstants.indexLimitPort);

  private Timer timer = new Timer();
  private Timer flyTimer = new Timer();
  private Timer pulseTimer = new Timer();
  private double lastShotTimestamp = -1;
  
  Phase currentPhase = Phase.ATTACK;
  Indexer currentIndexer = Indexer.OFF;
  Wheel currentWheel = Wheel.OFF;

  public final VelocityVoltage vel = new VelocityVoltage(0).withSlot(0);
  // Velocity controller for indexer (use same units as flywheel velocity)
  public final VelocityVoltage indexVel = new VelocityVoltage(0).withSlot(0);
  
  public MotorAlignmentValue MotorAlignment = MotorAlignmentValue.Aligned; // Aligned or Opposed
  TalonFXConfiguration configs = new TalonFXConfiguration();
  TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();

  public Storage() {

    configs.Slot0.kP = Constants.StorageConstants.wheelP;
    configs.Slot0.kI = Constants.StorageConstants.wheelI;
    configs.Slot0.kD = Constants.StorageConstants.wheelD;
    
    configs.CurrentLimits.StatorCurrentLimit = Constants.StorageConstants.wheelStatorLimit;
    configs.CurrentLimits.SupplyCurrentLimit = Constants.StorageConstants.wheelSupplyLimit;

  indexerConfigs.CurrentLimits.StatorCurrentLimit = Constants.StorageConstants.indexerStatorLimit;
  indexerConfigs.CurrentLimits.SupplyCurrentLimit = Constants.StorageConstants.indexerSupplyLimit;
  // indexer PID defaults
  indexerConfigs.Slot0.kP = Constants.StorageConstants.indexerP;
  indexerConfigs.Slot0.kI = Constants.StorageConstants.indexerI;
  indexerConfigs.Slot0.kD = Constants.StorageConstants.indexerD;
  // FF will be provided dynamically when commanding velocity
    
    indexMotor1.getConfigurator().apply(indexerConfigs);
    indexMotor2.getConfigurator().apply(indexerConfigs);

    flyWheel.getConfigurator().apply(configs);
    indexMotor2.setControl(new Follower(indexMotor1.getDeviceID(), MotorAlignment));

  // Dashboard tunables for pre-spin and pulsed feeding
  SmartDashboard.putNumber("Storage/PreSpinSeconds", 0.75);
  SmartDashboard.putNumber("Storage/MinInterShotSeconds", 0.65);
  SmartDashboard.putNumber("Storage/ShotPulseSeconds", 0.08);
  SmartDashboard.putNumber("Storage/WheelReadyRpsTol", 2.0);
  // Indexer velocity (same units as flywheel velocity measurement)
  SmartDashboard.putNumber("Storage/IndexerVelocity", Constants.StorageConstants.indexSpeed);
  SmartDashboard.putNumber("Storage/FirstShotsCount", 2);
  // Hold velocity between pulses to avoid RPM sag. 0 disables.
  SmartDashboard.putNumber("Storage/PulseHoldVelocity", 12.0);
  // Indexer PID/FF live tuning
  SmartDashboard.putNumber("Storage/IndexerP", Constants.StorageConstants.indexerP);
  SmartDashboard.putNumber("Storage/IndexerI", Constants.StorageConstants.indexerI);
  SmartDashboard.putNumber("Storage/IndexerD", Constants.StorageConstants.indexerD);
  SmartDashboard.putNumber("Storage/IndexerFF", Constants.StorageConstants.indexerFF);

  }

  public double getFlywheelSpeed(){
    return wheelSpeed;
  }

  public void delayMotorStart(){
    // Deprecated: pulsed feeding handled in periodic() now.
    return;
  }
  
public void moveWheel(double speed) {
  targetWheelSpeed = speed;
   if (getWheel() == Wheel.ON)
  { 
      flyWheel.setControl(vel.withVelocity(speed).withFeedForward(Constants.StorageConstants.wheelFF));
  }
  else
  {
    targetWheelSpeed = 0;
    flyWheel.setControl(new DutyCycleOut(0));
  }
}

public void moveIndexer(double speed) {
  targetIndexSpeed = speed;
   if (getIndexer() == Indexer.ON)
  { 
      // Set indexer to velocity control using the same controller type as the wheel
      indexMotor1.setControl(indexVel.withVelocity(speed));
      
  }
  else
  {
    // stop indexer
    indexMotor1.setControl(new DutyCycleOut(0));
  }
}

public Phase getPhase() {
  return currentPhase;
}

public void setPhase(Phase phase) {
  currentPhase = phase;
}

public Wheel getWheel() {
  return currentWheel;
}

public void setWheel(Wheel wheel) {
  currentWheel = wheel;
  if (wheel == Wheel.ON){
    spin = true;
  } else{
    spin = false;
  }
}

public void setIndexer(Indexer indexer) {
  currentIndexer = indexer;
  if (indexer == Indexer.ON){
    shoot = true;
  } else{
    shoot = false;
  }
}

public Indexer getIndexer() {
  return currentIndexer;
}


public void resetTimer(){
  timer.stop();
  timer.reset();
}
public void resetFlyTimer(){
  flyTimer.stop();
  flyTimer.reset();
}
  @Override
  public void periodic() {

    // Pulsed feeding logic:
    // - When indexer is ON and wheel is ON, wait pre-spin seconds after wheel enabled
    // - Require wheel RPM within tolerance of target
    // - Pulse indexer for ShotPulseSeconds, then wait MinInterShotSeconds before next pulse

    wheelSpeed = flyWheel.getRotorVelocity().refresh().getValueAsDouble();
    indexSpeed = indexMotor1.getRotorVelocity().refresh().getValueAsDouble();

  double preSpin = SmartDashboard.getNumber("Storage/PreSpinSeconds", 0.75);
  double minInter = SmartDashboard.getNumber("Storage/MinInterShotSeconds", 0.65);
  double pulseSec = SmartDashboard.getNumber("Storage/ShotPulseSeconds", 0.08);
  double rpsTol = SmartDashboard.getNumber("Storage/WheelReadyRpsTol", 2.0);
  double feedVel = SmartDashboard.getNumber("Storage/IndexerVelocity", Constants.StorageConstants.indexSpeed);
  // Read PID/FF and apply if changed (cheap to do each loop)
  double ixP = SmartDashboard.getNumber("Storage/IndexerP", Constants.StorageConstants.indexerP);
  double ixI = SmartDashboard.getNumber("Storage/IndexerI", Constants.StorageConstants.indexerI);
  double ixD = SmartDashboard.getNumber("Storage/IndexerD", Constants.StorageConstants.indexerD);
  double ixFF = SmartDashboard.getNumber("Storage/IndexerFF", Constants.StorageConstants.indexerFF);
  // Apply PID values to config and re-apply if they differ
  if (indexerConfigs.Slot0.kP != ixP || indexerConfigs.Slot0.kI != ixI || indexerConfigs.Slot0.kD != ixD) {
    indexerConfigs.Slot0.kP = ixP;
    indexerConfigs.Slot0.kI = ixI;
    indexerConfigs.Slot0.kD = ixD;
    indexMotor1.getConfigurator().apply(indexerConfigs);
    indexMotor2.getConfigurator().apply(indexerConfigs);
  }
  int firstCount = (int) SmartDashboard.getNumber("Storage/FirstShotsCount", 2);

    // If wheel is ON and we've just turned it ON, start flyTimer
    if (getWheel() == Wheel.ON) {
      if (!flyTimer.isRunning()) {
        flyTimer.start();
        // set first-shot count
        TurretUtil.setFirstShotsCount(firstCount);
      }
    } else {
      // wheel off: reset timers and indexer
      if (flyTimer.isRunning()) flyTimer.stop();
      flyTimer.reset();
      pulseTimer.stop();
      pulseTimer.reset();
      indexMotor1.setControl(new DutyCycleOut(0));
    }

    // Only allow feeding if indexer requested ON
    if (getIndexer() == Indexer.ON && getWheel() == Wheel.ON) {
      boolean wheelReady = Math.abs(targetWheelSpeed - wheelSpeed) <= rpsTol;
      boolean preSpinOk = flyTimer.hasElapsed(preSpin);

      double now = Timer.getFPGATimestamp();
      boolean canStartPulse = !pulseTimer.isRunning() && (lastShotTimestamp < 0 || (now - lastShotTimestamp) >= minInter);

        if (preSpinOk && wheelReady && canStartPulse) {
        // start pulse: set indexer to feed velocity for the pulse duration
        pulseTimer.reset();
        pulseTimer.start();
        indexMotor1.setControl(indexVel.withVelocity(feedVel).withFeedForward(ixFF));
      }

      // maintain or end pulse
      if (pulseTimer.isRunning()) {
        if (pulseTimer.hasElapsed(pulseSec)) {
          // end pulse
          pulseTimer.stop();
          pulseTimer.reset();
          // stop aggressive feed pulse (switch to hold or stop)
          double holdVel = SmartDashboard.getNumber("Storage/PulseHoldVelocity", 8.0);
          if (holdVel > 0) {
            indexMotor1.setControl(indexVel.withVelocity(holdVel).withFeedForward(ixFF));
          } else {
            indexMotor1.setControl(new DutyCycleOut(0));
          }
          lastShotTimestamp = Timer.getFPGATimestamp();
          TurretUtil.noteShotFired();
        }
      }
      else {
        // If not actively pulsing, optionally apply a low-velocity hold to reduce
        // motor RPM sag/brownout on some hardware. Configured via
        // Storage/PulseHoldVelocity (0 disables).
        double holdVel = SmartDashboard.getNumber("Storage/PulseHoldVelocity", 8.0);
        if (holdVel > 0) {
          // Only apply hold when indexer is requested and wheel is on but not
          // when actively pulsing or within inter-shot cooldown
          if (!pulseTimer.isRunning() && !canStartPulse) {
            indexMotor1.setControl(indexVel.withVelocity(holdVel).withFeedForward(ixFF));
          }
        }
      }
    }

    if(getWheel() == Wheel.ON) {
       moveWheel(SmartDashboard.getNumber("Target Wheel Speed", targetWheelSpeed));
    }
       
    SmartDashboard.putBoolean("Storage Limit Switch", indexLimit.get());
    SmartDashboard.putString("Phase", currentPhase.toString());
    SmartDashboard.putString("On Off", currentIndexer.toString());
    SmartDashboard.putNumber("Current Wheel Speed", wheelSpeed);
    SmartDashboard.putNumber("Current Indexer Speed", indexSpeed);
    SmartDashboard.putNumber("Target Indexer Speed", targetIndexSpeed);
    SmartDashboard.putBoolean("Wheel", spin);
    SmartDashboard.putBoolean("Shooting", shoot);
    SmartDashboard.putNumber("Target Wheel Speed", targetWheelSpeed);
    targetWheelSpeed = SmartDashboard.getNumber("Target Wheel Speed", targetWheelSpeed);

    
  }
}

