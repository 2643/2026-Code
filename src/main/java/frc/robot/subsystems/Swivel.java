// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;


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
  public double ty;
  private final String limelightName = "limelight-bhavik";
  private final String limelightURL = "http://10.26.43.201:5801/";
  public boolean isVisible;
  public double yaw;
  public double area;
  public double fiducialID;
  public States currentState = States.INITIALIZING;
  public Mode currentMode = Mode.MANUAL;

  
  TalonFXConfiguration configs = new TalonFXConfiguration();

 public Swivel() {
    configs.Slot0.kP = Constants.TurretConstants.swivelP;
    configs.Slot0.kI = Constants.TurretConstants.swivelI;
    configs.Slot0.kD = Constants.TurretConstants.swivelD;

    configs.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.swivelAccel;
    configs.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.swivelVel;

    configs.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.swivelStatorLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.swivelSupplyLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    swivelMotor.getConfigurator().apply(configs);
    swivelMotor.setNeutralMode(NeutralModeValue.Brake);
    setSwivelPos(0);
  }

   public void moveSwivel(double pos) {
    swivelTarget = pos;
    swivelMotor.setControl(new MotionMagicVoltage(pos));
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
    if (ty > 0) {
      dist = Math.log(ty)/600*5/2*5/1.25;
      // if (dist >= 0.2) {
      //   dist = 0.2;
      // }
    }
    else if (ty < 0) {
      dist = -(Math.log(-ty)/600/2*5*5/1.25);
      // if (dist <= -0.2) {
      //   dist = -0.2;
      // }
    }
    return dist;
  }

  // Convert a Limelight tx (degrees or normalized) into a small swivel rotation offset.
  // This scale is intentionally small; tune on robot.
  private double txToOffset(double txVal) {
    return txVal * 0.01; // scale factor: 0.01 rotations per degree (tune as needed)
  }

  
   public void autoAlign(){
     if (currentMode == Mode.AUTOAIM) {
        if (isVisible == true && currentState == States.INITIALIZED) {
          moveSwivel(getSwivelPos()+getDist());
        }
        // duty code
        // else if (isVisible == true && tx>0 && isLimitedX1 == true && isLimitedX2 == true && isLocked == false) {
        //   swivelMotor.setControl(new DutyCycleOut(fullReverseRotation()));
        // }
        else {
          moveSwivel(getSwivelPos());
        }
      }
    }
   

  // public void autoAlign(){
  //   if (currentMode != Mode.AUTOAIM) {
  //     return;
  //   }

  //   // Read raw fiducials from Limelight (gives id and txnc)
  //   var fiducials = LimelightHelpers.getRawFiducials(limelightName);
  //   int n = fiducials.length;

  //   // Determine current alliance and phase (attack/defense)
  //   var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  //   var phase = RobotContainer.m_Storage.getPhase();

  //   // Helper method defined at class scope: txToOffset

  //   if (isVisible && currentState == States.INITIALIZED) {
  //     if (n >= 2) {
  //       // Map ids -> tx
  //       Map<Integer, Double> map = new HashMap<>();
  //       for (var f : fiducials) {
  //         map.put(f.id, f.txnc);
  //       }

  //       // choose candidate pairs depending on phase+alliance
  //       int[][] pairs;
  //       if (phase == frc.robot.subsystems.Storage.Phase.DEFENSE) {
  //         if (alliance == DriverStation.Alliance.Red) {
  //           pairs = new int[][]{{6,4},{4,1}};
  //         } else {
  //           pairs = new int[][]{{17,20},{20,22}};
  //         }
  //       } else { // ATTACK
  //         if (alliance == DriverStation.Alliance.Red) {
  //           pairs = new int[][]{{8,10},{10,11}};
  //         } else {
  //           pairs = new int[][]{{27,26},{26,24}};
  //         }
  //       }

  //       // Search for a matching pair we can aim between
  //       for (var pair : pairs) {
  //         if (map.containsKey(pair[0]) && map.containsKey(pair[1])) {
  //           double txA = map.get(pair[0]);
  //           double txB = map.get(pair[1]);
  //           double meanTx = (txA + txB) / 2.0;
  //           double offset = txToOffset(meanTx);
  //           moveSwivel(getSwivelPos() + offset);
  //           return;
  //         }
  //       }

  //       // No targeted pair found: fallback to average of all detections
  //       double sum = 0;
  //       for (var f : fiducials) sum += f.txnc;
  //       double avg = sum / n;
  //       moveSwivel(getSwivelPos() + txToOffset(avg));
  //       return;
  //     } else if (n == 1) {
  //       var f = fiducials[0];
  //       double baseOffset = txToOffset(f.txnc);
  //       if (phase == frc.robot.subsystems.Storage.Phase.ATTACK) {
  //         // In attack phase, simply aim at the detected tag
  //         moveSwivel(getSwivelPos() + baseOffset);
  //         return;
  //       } else {
  //         // Defense: apply small adjustments based on tag ID
  //         double tweak = 0.05; // rotation tweak; tune on robot
  //         int id = f.id;
  //         if (id == 17 || id == 1) {
  //           // shoot a little to the right
  //           moveSwivel(getSwivelPos() + baseOffset + Math.abs(tweak));
  //           return;
  //         } else if (id == 20 || id == 4) {
  //           // shoot a little to whatever side you're on (use sign of tx)
  //           moveSwivel(getSwivelPos() + baseOffset + Math.signum(f.txnc) * tweak);
  //           return;
  //         } else if (id == 22 || id == 6) {
  //           // shoot a little to the left
  //           moveSwivel(getSwivelPos() + baseOffset - Math.abs(tweak));
  //           return;
  //         } else {
  //           // Unknown tag: just aim at it
  //           moveSwivel(getSwivelPos() + baseOffset);
  //           return;
  //         }
  //       }
  //     }
  //   }

    // Default fallback: manual turret position
  //   moveSwivel(Constants.TurretConstants.manualTurret);
  // }

    @Override
  public void periodic() {
    autoAlign();
    ty = LimelightHelpers.getTYNC(limelightName);
    isVisible = LimelightHelpers.getTV(limelightName);
    area = LimelightHelpers.getTA(limelightName);
    fiducialID = LimelightHelpers.getFiducialID(limelightName);
  
    SmartDashboard.putNumber("Current Swivel Position", getSwivelPos());
    SmartDashboard.putNumber("Target Swivel Position", swivelTarget);
    SmartDashboard.putNumber("Distance", getDist());
    SmartDashboard.putNumber("Target Yaw", ty);
    SmartDashboard.putString("Current State", currentState.toString());
    SmartDashboard.putBoolean("Swivel Limit", getSwivelLimit());
    SmartDashboard.putString("Current Mode", getMode().toString());
    SmartDashboard.putBoolean("Apriltag", isVisible);
;
    // doesn't work when initializing
    //  if (getSwivelPos() > Constants.TurretConstants.swivelSoftLimit1) {
    //   moveSwivel(Constants.TurretConstants.swivelSoftLimit1 - 0.1);
    // } else if (getSwivelPos() < Constants.TurretConstants.hoodSoftLimit2) {
    //   moveSwivel(Constants.TurretConstants.swivelSoftLimit2 + 0.1);
    // } else if (getSwivelPos() >= Constants.TurretConstants.swivelHardLimit1 || getSwivelPos() <= Constants.TurretConstants.swivelHardLimit2) {
    //   swivelMotor.disable();
    // }
  }
}
