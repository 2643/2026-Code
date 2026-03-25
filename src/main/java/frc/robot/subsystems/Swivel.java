// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Storage.Phase;
import frc.robot.util.LimelightHelpers;
import java.util.ArrayList;

public class Swivel extends SubsystemBase {

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED
  }

  public enum Mode {
    AUTOAIM,
    MANUAL
  }

  public static TalonFX swivelMotor = new TalonFX(0); 
  public DigitalInput swivelLimit = new DigitalInput(Constants.TurretConstants.swivelLimitPort);
  public static double swivelTarget;
  public double dist;
  public double tx;
  private final String limelightName = "limelight-allen";
  private final String limelightURL = "http://10.26.43.201:5801/";
  public boolean isVisible;
  public double yaw;
  public double area;
  public double fiducialID;
  public States currentState = States.INITIALIZING;
  public Mode currentMode = Mode.MANUAL;
  public java.util.Set<Integer> seen = new java.util.HashSet<>();
  public ArrayList<Double> tags = new ArrayList<Double>();
  public Timer timer = new Timer();


  
  TalonFXConfiguration configs = new TalonFXConfiguration();

 public Swivel() {
    var magicmotionconfig = configs.MotionMagic;
    configs.Slot0.kP = Constants.TurretConstants.swivelP;
    configs.Slot0.kI = Constants.TurretConstants.swivelI;
    configs.Slot0.kD = Constants.TurretConstants.swivelD;

    magicmotionconfig.MotionMagicAcceleration = Constants.TurretConstants.swivelAccel;
    magicmotionconfig.MotionMagicCruiseVelocity = Constants.TurretConstants.swivelVel;

    configs.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.swivelStatorLimit;
    configs.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.swivelSupplyLimit;

    swivelMotor.getConfigurator().apply(configs);
    swivelMotor.setNeutralMode(NeutralModeValue.Brake);
    setSwivelPos(0);
  }

   public void moveSwivel(double target) {
    // target += Constants.TurretConstants.antiRotationOffset;
    if (target < Constants.TurretConstants.swivelHardLimit1 && target > Constants.TurretConstants.swivelHardLimit2){
          swivelTarget = target;
          swivelMotor.setControl(new MotionMagicVoltage(target));
    }
  }

  public Mode getMode() {
    return currentMode;
  }

  public void setMode(Mode mode) {
    currentMode = mode;
  }
  
  public States getState() {
    return currentState;
  }

   public void startTimer() {
      timer.start();
  }

  public void resetTimer() {
    timer.stop();
    timer.reset();
  }

  public void setState(States state) {
    currentState = state;
  }
  
  public double getSwivelPos(){
    return swivelMotor.getPosition().getValueAsDouble();
  }

  public void setSwivelPos(double pos) {
    swivelMotor.setPosition(pos);
  }

  public boolean getSwivelLimit(){
    return swivelLimit.get();
  }

    public double getDist() {
    if (tx > 0) {
      dist = Math.log(tx)/600*5/2*6/1.25;
    }
    else if (tx < 0) {
      dist = -(Math.log(-tx)/600/2*5*6/1.25);
    }
    return dist;
  }

   public void autoAlign(){
    var fiducials = LimelightHelpers.getRawFiducials(limelightName);
    for (var f : fiducials) {
        seen.add(f.id);
    }

    if(currentState == States.INITIALIZED && currentMode == Mode.AUTOAIM) {
     if (RobotContainer.m_Storage.getPhase() == Phase.ATTACK) {
        if (isVisible == true && (seen.contains(10) || seen.contains(26)|| seen.contains(11)|| seen.contains(8)|| seen.contains(24)|| seen.contains(27))) 
          moveSwivel(getSwivelPos()+getDist());
        else 
          moveSwivel(getSwivelPos());
      }
      

      if (RobotContainer.m_Storage.getPhase() == Phase.DEFENSE) {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              if (isVisible == true && (seen.contains(1) || seen.contains(4) || seen.contains(5) || seen.contains (23) || seen.contains(27))) 
                moveSwivel(getSwivelPos()+getDist()+0.1);
              else if (isVisible == true && (seen.contains(2)|| seen.contains(6) || seen.contains(24) || seen.contains(28))) 
                moveSwivel(getSwivelPos()+getDist()-0.1);
        } else {
          if (isVisible == true && (seen.contains(17) || seen.contains(20) || seen.contains(21) || seen.contains(11) || seen.contains(7)))  
            moveSwivel(getSwivelPos()+getDist()+0.1);
          else if (isVisible == true && (seen.contains(22)|| seen.contains(18) || seen.contains(12) || seen.contains(8))) 
            moveSwivel(getSwivelPos()+getDist()-0.1);
        }
      }
    }

    for (int id : seen) {
      tags.add((double)id);
    } 

    SmartDashboard.putNumberArray("Seen", tags.stream().mapToDouble(Double::doubleValue).toArray());


      seen.clear();
      tags.clear();
    }
   
    @Override
  public void periodic() {

    tx = LimelightHelpers.getTYNC(limelightName);  
    isVisible = LimelightHelpers.getTV(limelightName);
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
    SmartDashboard.putNumber("Current Swivel Position", getSwivelPos());
    SmartDashboard.putNumber("Target Swivel Position", swivelTarget);
    SmartDashboard.putNumber("dist", getDist());
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putBoolean("AprilTag", isVisible);
    SmartDashboard.putString("Current State", currentState.toString());
    SmartDashboard.putBoolean("Swivel Limit", getSwivelLimit());
    SmartDashboard.putString("Current Mode", getMode().toString());

    autoAlign();
    if(getState() == States.INITIALIZED) {
     if (getSwivelPos() > Constants.TurretConstants.swivelSoftLimit1) {
      moveSwivel(Constants.TurretConstants.swivelSoftLimit1 - 0.1);
    } else if (getSwivelPos() < Constants.TurretConstants.swivelSoftLimit2) {
      moveSwivel(Constants.TurretConstants.swivelSoftLimit2 + 0.1);
    } else if (getSwivelPos() >= Constants.TurretConstants.swivelHardLimit1 || getSwivelPos() <= Constants.TurretConstants.swivelHardLimit2) {
      // swivelMotor.disable();
    }
    }
  }
}
