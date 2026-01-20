// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.Vision.Followtag;
import frc.robot.Commands.Vision.PoseAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    public final static CommandXboxController driver = new CommandXboxController(0);
    public final static CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Subsystems
    Autos autos = new Autos(drivetrain);
    final Shooter shooter = new Shooter();
    Turret turret = new Turret();

    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);



    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private SendableChooser<Command> autoChooserManual;
    public static SendableChooser<String> lazyAuto2000 = new SendableChooser<String>();
    public static SendableChooser<Integer> autoDriveLocation = new SendableChooser<Integer>();

    
    public RobotContainer() {

        autos.makeNamedCommands();
    
        autoChooserManual = new SendableChooser<Command>();
        autoChooserManual = autos.configureChooser(autoChooserManual);

        autoChooser = AutoBuilder.buildAutoChooser("Line");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("lazyAuto2000", lazyAuto2000);

        SmartDashboard.putData("SmartPathfinding", autoDriveLocation); 

        SmartDashboard.putData("Auto Mode 2000", autoChooserManual);

        configureBindings();
    }

    private void configureBindings() {
        if (drivetrain != null) {
            // Set up all commands and button bindings here
            // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        

        driver.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.povDown().whileTrue(new PoseAlign(drivetrain));
        driver.rightBumper().whileTrue(new Followtag(drivetrain));//, () -> -joystick.getLeftX() * MaxSpeed, () -> -joystick.getLeftY() * MaxSpeed));
        
        // driver.rightTrigger().whileTrue(new InstantCommand( () -> shooter.variableShot(0.1)));
        driver.leftTrigger().whileTrue(new InstantCommand( () -> turret.rotateTurret(-0.1)));
        driver.rightTrigger().whileTrue(new InstantCommand( () -> turret.rotateTurret(0.1)));
        driver.rightTrigger().whileFalse(new InstantCommand( () -> turret.rotateTurret(0)));
        driver.leftTrigger().whileFalse(new InstantCommand( () -> turret.rotateTurret(0)));


        drivetrain.registerTelemetry(logger::telemeterize);
        }
    }

    
    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }



    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

}
