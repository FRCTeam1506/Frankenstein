// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.slot0Configs;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  //Auto Aim To Hub 
  Pose2d vRobotPose;
  double latency = 0.15; // Tuned constant
  Translation2d futurePos;
  Translation2d goalLocation = new Translation2d(5.229, 3.431);
  Translation2d targetVec;
  double dist;
  double idealHorizontalSpeed = 1; //double idealHorizontalSpeed = ShooterTable.getSpeed(dist);
  Translation2d robotVelVec;
  Translation2d shotVec;
  double finalTurretAngle;
  double newHorizontalSpeed;
  double ticksPerDegree;


  //double idealSpeed = getShooterSpeedForDistance(distance);
  

  boolean red;
  Optional<Alliance> alliance = DriverStation.getAlliance();
  public static int shootMode = 0; //1 = keep heading at 0. 2 = Main shoot to goal. 3 = mail left. 4 = mail right.

  //Variables for getting angle to goal.
  double goalX = 5.229;
  double goalY = 3.431;
  double theta;
  double angleToGoal;
  double turretAngleTarget;
  double finalTurretPos;
  double vRobotY;
  double vRobotX;

  //MAILING
  //RED
  //RIGHT
  final double goalRightRedY = 0;
  final double goalRightRedX = 0;
  //LEFT
  final double goalLeftRedY = 0;
  final double goalLeftRedX = 0;

  //BLUE
  //RIGHT
  final double goalRightBlueY = 0;
  final double goalRightBlueX = 0;
  //LEFT
  final double goalLeftBlueY = 0;
  final double goalLeftBlueX = 0;

  //InterpolatingDoubleTreeMap turretPos = new InterpolatingDoubleTreeMap();
  double heading;
  double turretAngle;


  double testAngle;
  
  /** Creates a new Turret. */
    private TalonFX Turret = new TalonFX(TurretConstants.TURRET_ID);
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
    
    // private Encoder encoder = new Encoder(0, 0);

  public Turret() {
    if (alliance.get() == Alliance.Red) {
      red = true;
    } else {
      red = false;
    }

    var talonFXConfigs = new TalonFXConfiguration();

    // talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Turret.getConfigurator().apply(config);

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2.5; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    config.Slot0 = slot0Configs;


    Turret.getConfigurator().apply(motionMagicConfigs);
    Turret.getConfigurator().apply(slot0Configs); 


    // turretPos.put(0.0, -0.973633);
    // turretPos.put(90.0, 2.453125);
    // turretPos.put(180.0, 5.595215);
    // turretPos.put(270.0, 9.096680);
  }

  public void rotateTurret(double turretSpeed) {
    Turret.set(turretSpeed);
  }

  public void setTurretPos(double pos) {
    Turret.setPosition(pos);
  }

  public void zeroTurret() {
    Turret.setControl(m_motmag.withPosition(0));
  }

  public void fixedTurretPosition() {
    // heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
    // turretAngleTarget = 0 - heading;
    // finalTurretPos = turretAngleTarget * 13.2;
    // Turret.setControl(m_motmag.withPosition(finalTurretPos));
    // System.out.println(finalTurretPos);
  }


  public void turretPosition(double goalX, double goalY) {
      // theta = Math.atan((this.goalY - vRobotY) / (this.goalX - vRobotX));
      // angleToGoal = 90 - theta;
      // heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
      // turretAngleTarget = angleToGoal - heading;
      // finalTurretPos = turretAngleTarget * 13.2;
      // Turret.setControl(m_motmag.withPosition(finalTurretPos));
      // System.out.println(finalTurretPos);
  }

  public void shootModeChange(boolean up) {
    if (up == true) {
      shootMode += 1;
    } else {
      shootMode -= 1;
    }
  }

  public void setTurretToAngle (double angle) {
    Turret.setPosition((angle / 360) * 13.5); //Turret angle from degree to ticks.
    System.out.println("set Turret angle to" + angle);
  }

  @Override
  public void periodic() {
    // targetVec = goalLocation.minus(futurePos);
    // dist = targetVec.getNorm();
    // robotVelVec = new Translation2d(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
    // shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);
    // finalTurretAngle = shotVec.getAngle().getDegrees();
    // newHorizontalSpeed = shotVec.getNorm();
    // System.out.println(finalTurretAngle);
    
    // setTurretToAngle(90 - Math.atan((goalY - drivetrain.getState().Pose.getY()) / (goalX - drivetrain.getState().Pose.getX())));
    // testAngle = 90 - Math.atan((goalY - drivetrain.getState().Pose.getY()) / (goalX - drivetrain.getState().Pose.getX()));
    // System.out.println("Angle" + testAngle);

    //System.out.println(shootMode); 
    heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
    // turretAngleTarget = 0 - heading;
    // finalTurretPos = turretAngleTarget * 13.2;
    // Turret.setControl(m_motmag.withPosition(finalTurretPos));
    // System.out.println(finalTurretPos);
    vRobotX = drivetrain.getState().Pose.getX() + (drivetrain.getState().Speeds.vxMetersPerSecond /* * latency*/);//times latency??
    vRobotY = drivetrain.getState().Pose.getY()+ (drivetrain.getState().Speeds.vyMetersPerSecond /* * latency*/);//times latency??;
    theta = Math.atan((goalY - vRobotY) / (goalX - vRobotX));



    
    // switch (shootMode) {
    //   case 1:
    //     turretAngleTarget = 0 - heading;
    //     finalTurretPos = turretAngleTarget * 13.2;
    //     Turret.setControl(m_motmag.withPosition(finalTurretPos));
    //     System.out.println(finalTurretPos);
    //     break;

    //   case 2:
    //     turretPosition(goalX, goalY);
    //     break;

    //   case 3:
    //     if (red == true) {
    //         turretPosition(goalLeftRedY, goalLeftRedX);
    //     }
    //     else {
    //       turretPosition(goalLeftBlueY, goalLeftBlueX);
    //     }
    //     break;
    //   case 4:
    //     if (red == true) {
    //       turretPosition(goalRightRedY, goalRightRedX);
    //     }
    //     else {
    //       turretPosition(goalRightBlueY, goalRightBlueX);
    //     }
    //     break;
    // }

  //   //add some sort of variable that gives the turret angle based on the encoder position
  //   //turretAngle = (encoder.getDistance() / 4096) * 360;

  //   //add some sort of function such that the turret is constantly facing the hub
  //   /*Facing Hub Function
  //    * 
  //    * 
  //    */

  //   // This method will be called once per scheduler run


  //   //Math to get turret angle to goal
  //   // //theta = Math.atan2((drivetrain.getState().Pose.getY() - goalY), (drivetrain.getState().Pose.getX() - goalX));
  //   // theta = Math.atan((goalY - vRobotY) / (goalX - vRobotX));
  //   // angleToGoal = 90 - theta;
  //   // heading = drivetrain.getPigeon2().getYaw().getValueAsDouble() / 360;
  //   // double turretAngleTarget = angleToGoal - heading;
  //   // double finalTurretPos = turretAngleTarget * 13.2;
  //   // Turret.setControl(m_motmag.withPosition(finalTurretPos));
  //   // System.out.println(finalTurretPos);

    if (shootMode > 4) {
      shootMode = 4;
    } else if (shootMode < 0) {
      shootMode = 0;
    }
    // if (Turret.getPosition().getValueAsDouble() > 6.5 || Turret.getPosition().getValueAsDouble() < -6.5) {
    //   Turret.set(0);
    // }
  }
}
