// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {
  private TalonFX leftIntake = new TalonFX(IntakeConstants.LEFTINTAKE_ID);
  private TalonFX rightIntake = new TalonFX(IntakeConstants.LEFTINTAKE_ID);


  /** Creates a new Intake. */
  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    leftIntake.getConfigurator().apply(config);
    rightIntake.getConfigurator().apply(config);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
