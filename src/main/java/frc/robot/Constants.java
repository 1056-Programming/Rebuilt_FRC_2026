package frc.robot;

public class Constants {
    public static final double kTranslationLimiter = 0.05;

    public class Shooter {
        // REV SparkFlex 
        public static final int kLeftBackspinID = 33;
        public static final int kRightBackspinID = 34;
        
        // Pheonix Krakens
        public static final int kRightShootingID = 20;
        public static final int kMiddleShootingID = 21;
        public static final int kLeftShootingID = 22;

        // PID Controller
        public static final double kShooterP = 0.12; // kinda good P is at value 0.1
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;

        public static final double kBackP = 0.005;
        public static final double kBackI = 0;
        public static final double kBackD = 0;

        // IDS
        public static final int[] validIDs = {9,10,26,25};
    }

    public class Indexor {
        // REV SparkMax 
        public static final int kIndexorID = 32;
        public static final int kConveyorID = 30;
    }

    public class Intake{
        // REV SparkFLex 
        public static final int kIntakeID = 35;

        // REV SparkMax
        public static final int kPivotID = 31;

        // PID values
        public static final double kIntakeP = 0.01;
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
    
    }

    public class Vision{
        // Vision threshold: 
        public static final double ambiguityThreshold = 0.5;

        // Vision distance to camera threshold: 
        public static final double distanceThreshold = 5; 
    }

}
