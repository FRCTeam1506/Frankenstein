// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.slot0Configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  double turretAngle;
  
  /** Creates a new Turret. */
    private TalonFX Turret = new TalonFX(TurretConstants.TURRET_ID);
    // private Encoder encoder = new Encoder(0, 0);


  public Turret() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // config.Slot0 = Constants.slot0Configs;

    Turret.getConfigurator().apply(config);
  }

  public void rotateTurret(double turretSpeed) {
    Turret.set(turretSpeed);
  }

  public void setTurretPos(double pos) {
    Turret.set
  }

  @Override
  public void periodic() {
    //add some sort of variable that gives the turret angle based on the encoder position
    // turretAngle = (encoder.getDistance() / 4096) * 360;

    //add some sort of function such that the turret is constantly facing the hub
    /*Facing Hub Function
     * 
     * 
     */

    // This method will be called once per scheduler run
  }
}
