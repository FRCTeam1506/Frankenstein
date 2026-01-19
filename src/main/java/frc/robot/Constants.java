package frc.robot;

import java.util.HashMap;
import java.util.Map;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.DigitalInput;
// import frc.robot.generated.TunerConstants;


public class Constants {

    public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);

    public class SwerveConstants {

        public static final double driveKP = 1.5;
        public static final double driveKI = 0.15; //0
        public static final double driveKD = 0.175;//0.075

        public static final double alignKP = 1.5; //1.5
        public static final double alignKI = 0.15;//0
        public static final double alignKD = 0.175;//0.075
        public static final double dMaxVelocity = 1;
        public static final double dMaxAccel = 2;

        public static final double tMaxVelocity = 4; //rad/s
        public static final double tMaxAccel = 1;

        //on the fly path constraitns
        public static PathConstraints oTF_Constraints = new PathConstraints(5.3, 5, Math.toRadians(270), Math.toRadians(360));    
    }

    public class ShooterConstants {
        public static final int LEFTSHOOTER_ID = 0; //Yet to be set
        public static final int RIGHTSHOOTER_ID = 0; //Yet to be set
    }
    public class TurretConstants {
        public static final int TURRET_ID = 55; //Yet to be set
        public static final int ENCODER_ID = 0; //Yet to be set 
    }

    public class CandleConstants {
        public static final int CANDLE_ID = 62;
        public static boolean noah = false;
    }





    public class VisionConstants {

        public static final String LL_LEFT = "limelight-left";
        public static final String LL_CENTER = "limelight-center";
        public static final String LL_FRONT = "limelight-front"; //human player side

        // public static final double coralStationLeftHeading = 235;
        // public static final double coralStationRightHeading = 125;   
        
        
        //todo: definitely change these values for our robot.
        // X is in the normal direction of the tag, Y is parallel to the tag 
        public static final Transform2d leftBranch = new Transform2d(0.23769, -0.35, new Rotation2d(Math.toRadians(-2.8))); // 0.237 x perfect for left branch after states // -0.46769 x, //math.pi puts the ramp touching the reef //-4 DEGREES good at FRCC, but too angled for SVSU hemlock, reduced to -2.8.
        public static final Transform2d rightBranch = new Transform2d(0.23769, 0.0, new Rotation2d(Math.toRadians(-2.6))); //both used to be 0 degrees, but -4 is (italian chef kiss)
        public static final Transform2d reefAlgae = new Transform2d(0.5,0.0,new Rotation2d(0));


        public static double std02 = 15; //standard deviation for vision??? seems high
        public static double maxStdDeviation = .5; 
        /** cut-off tag area, don't trust past this point */
        public static final double minTagArea = 0.35; 
        public static double visionStdSlope = (maxStdDeviation-std02)/(4-.2); // from .2 to 4 // 2m to .5m // units (Deviation / Tag Area)
        public static double visionStdConstant = std02 - visionStdSlope * .2; // Units (Deviation)
    
        public static double getVisionStd(double tagArea){
            double std = visionStdSlope * tagArea + visionStdConstant;
    
            if (tagArea < minTagArea){ 
                return 9999999;
            }
            if (std < maxStdDeviation){
                return maxStdDeviation;
            }
            
            return std;
        }

    }

   

    
        

}
