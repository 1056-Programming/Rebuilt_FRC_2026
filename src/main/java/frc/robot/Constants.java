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
        public static final int kShooterP = 0;
        public static final int kShooterI = 0;
        public static final int kShooterD = 0;
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
    }

    public static final double distanceThreshold = 3.0; // meters
    public static final double ambiguityThreshold = 0.2; // 0 to 1, where 0 is no ambiguity and 1 is high ambiguity
}
