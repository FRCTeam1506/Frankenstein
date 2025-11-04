//everything will work. Josh worked his magic.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.Instant;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Autos {
    

    static CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // enum autos { 
    //     Nothing, 
    //     Calibration10, 
    //     CalibrationSquare 
    //      }


    public Autos(CommandSwerveDrivetrain drivetrain){
        
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands(){
        

    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        chooser.addOption("Left", new PathPlannerAuto("Test"));
        

        return chooser; 
    }


}
//everything will work. Josh worked his magic.