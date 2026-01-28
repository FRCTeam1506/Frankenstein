// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooter = new TalonFX(ShooterConstants.LEFTSHOOTER_ID);
  private TalonFX rightShooter = new TalonFX(ShooterConstants.RIGHTSHOOTER_ID);

  /** Creates a new Shooter. */
  public Shooter() {
    //Configuration of the shooting motors
    TalonFXConfiguration config = new TalonFXConfiguration();

    // config.Slot0 = Constants.slot0Configs;
    
    //Use if it makes sense
    leftShooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
   rightShooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    

    leftShooter.getConfigurator().apply(config);
    rightShooter.getConfigurator().apply(config);

    

  }

  public void shootMax(){
    leftShooter.set(1);
    rightShooter.set(-1);
    System.out.println("MAX SHOT");
  }

  public void variableShot(double power){
    leftShooter.set(power);
    rightShooter.set(-power);
    System.out.println("VARIABLE SHOT");
  }
  public void shooterStop(){
    leftShooter.set(0);
    rightShooter.set(0);
    System.out.println("STOP SHOOTER");
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
