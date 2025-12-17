package frc.robot.Commands.Vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import javax.naming.spi.DirStateFactory.Result;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class OnlyTurn2Deg extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose;

  SwerveRequest.ApplyRobotSpeeds request;

  private boolean running = false;
  private Timer timer;
  double closeVelocityBoost = 0.0;
  double timeout = 10;
  int tagId;

  //private final ProfiledPIDController thetaController = new ProfiledPIDController(0.55,0, 0.003, new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));
  private final PIDController turnController = new PIDController(0.08, 0, 0);
  //double goalAngle;

  public DoubleSupplier leftY;
  public DoubleSupplier leftX;

  


  public OnlyTurn2Deg(CommandSwerveDrivetrain drivetrain, DoubleSupplier leftX, DoubleSupplier leftY) {
    this.drivetrain = drivetrain;
    this.timer = new Timer();
    this.leftX = leftX;
    this.leftY = leftY;
    addRequirements(drivetrain);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    request = new SwerveRequest.ApplyRobotSpeeds();
  }


  @Override
  public void initialize() {
    // Pose2d currentPose = drivetrain.getState().Pose;
    //Pose2d currentPose = new Pose2d(Vision.align3d_x, Vision.align3d_y, new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_CENTER))));
    
    // tagId = (int) LimelightHelpers.getFiducialID(VisionConstants.LL_CENTER);

    // if(tagId == -1){
    //   tagId = 0;
    // }

    //goalAngle = Vision.angles[tagId];

    // thetaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
    // thetaController.setGoal(goalAngle);
    // thetaController.setTolerance(1.5);

    // thetaController.reset(Math.toRadians(findCoterminalAngle(currentPose.getRotation().getDegrees()))); //still revolving before derevolving
    //thetaController.reset(0);

    this.timer.restart();
  }

  @Override
  public void execute() {
    running = true;
    
    // 1. Get the CURRENT tx value every frame (20ms)
    double currentTx = LimelightHelpers.getTX(VisionConstants.LL_CENTER);
    
    // 2. Get the CURRENT joystick values
    double xSpeed = leftX.getAsDouble();
    double ySpeed = leftY.getAsDouble();

    // 3. Calculate rotation based on the fresh tx
    double rotationOutput = 0;
    if (LimelightHelpers.getTV(VisionConstants.LL_CENTER)) {
        rotationOutput = turnController.calculate(currentTx, 0);
    }

    // 4. Pass the DOUBLES (not the Suppliers) to ChassisSpeeds
    // Note: Use field-relative or robot-relative based on your preference
    // if (ySpeed < 0.1 && xSpeed < 0.1) {
    //   drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, rotationOutput)));
    // } else {
      drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(ySpeed, xSpeed, rotationOutput)));
    //}
  }

  @Override
  public boolean isFinished() {
    // return thetaController.atGoal() || !LimelightHelpers.getTV(VisionConstants.LL_CENTER);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    running = false;
  }

  //chatgpt
  public static double findCoterminalAngle(double angle) {
    // Normalize the angle by subtracting or adding multiples of 360
    double normalizedAngle = angle % 360;
    
    // If the normalized angle is negative, add 360 to make it positive
    if (normalizedAngle < 0) {
        normalizedAngle += 360;
    }
    
    return normalizedAngle;
}

}