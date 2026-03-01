package frc.robot;

public class Constants {
    public static final double kTranslationLimiter = 0.05;

    public class Shooter {
        // REV SparkFlex 
        // public static final int kLeftBackspinID = 0;
        public static final int kBackSpinID = 34;
        
        // Pheonix Krakens
        public static final int kRightShootingID = 20;
        public static final int kMiddleShootingID = 21;
        public static final int kLeftShootingID = 22;

        // PID Controller
        public static final double ShooterKP = 0.066; // 0.066 is good
        public static final double ShooterKI = 0.0;
        public static final double ShooterKD = 0.0;

        public static final double ShooterKS = 0.05; 
        public static final double ShooterKV = 0.008; 
        public static final double ShooterKA = 1; 


        public static final double BackSpinKP = 0.004;
        public static final double BackSpinKI = 0;
        public static final double BackSpinKD = 0;

        // IDS
        public static final int[] validIDs = {9,10,26,25};
    }

    public class Indexor {
        // REV SparkMax 
        public static final int kIndexorID = 33;
        public static final int kConveyorID = 30;
    }

    public class Intake{
        // REV SparkFLex 
        public static final int kIntakeID = 35;

        // REV SparkMax
        public static final int kPivotID = 31;

        // PID values
        public static final double kIntakeP = 0.005;
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
    
    }

    public class Vision {
        // Vision threshold: 
        public static final double ambiguityThreshold = 0.5;

        // Vision distance to camera threshold: 
        public static final double distanceThreshold = 5; 
    }

    public class Swerve {
        public static final double kPositiveSlew = 0.5;
        public static final double kNegativeSlew = -1;
        public static final double kStart = 0; 

        // public static final double kTranslationP;
        // public static final double kTranslationI;
        // public static final double kTranslationD;

        
        // public static final double kRotationP;
        // public static final double kRotationI;
        // public static final double kRotationD;
    }

}
