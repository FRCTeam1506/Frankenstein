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

    public static double scoreSpeed = 0.8;
    public static boolean raisable;
    

    public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);





    

    public class SwerveConstants {

        public static final double driveKP = 1.5;
        public static final double driveKI = 0;
        public static final double driveKD = 0.075;

        public static final double alignKP = 1.5; //1.5
        public static final double alignKI = 0;
        public static final double alignKD = 0.075;

        public static final double dMaxVelocity = 1;
        public static final double dMaxAccel = 2;

        public static final double tMaxVelocity = 4; //rad/s
        public static final double tMaxAccel = 1;

        //on the fly path constraitns
        public static PathConstraints oTF_Constraints = new PathConstraints(5.3, 5, Math.toRadians(270), Math.toRadians(360));    
    }

    public class CandleConstants {
        public static final int CANDLE_ID = 62;
        public static boolean noah = false;
    }







    
        

}
