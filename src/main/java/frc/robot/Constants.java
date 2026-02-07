package frc.robot;

public class Constants {
    public static final double kTranslationLimiter = 0.05;

    public class Shooter {
        public static final int kBackspinID = 10;
        public static final int kShootingID = 11;

        public static final int kIndexerID = 12;
        public static final int kConveyorID= 13;
    }

    public class Indexing {
        public static final int kIndexerID = 12;
        public static final int kConveyorID= 13;
    }

    public class Intake{
        public static final int kLeft = 12;
        public static final int kRight= 13;
    }




    public static final String[] limelightNames = {"limelight", "limelight-apriltag", "limelight-megatag2" };
    public static final double ambiguityThreshold = 0.7;
    public static final double distanceThreshold = 3;
}
