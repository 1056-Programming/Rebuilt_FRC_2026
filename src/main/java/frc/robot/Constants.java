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
}
