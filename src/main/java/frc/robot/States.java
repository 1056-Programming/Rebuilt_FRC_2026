package frc.robot;

public class States {
    // Indexer states with corresponding indexer and conveyor speeds
    // Both are in motor speed range of -1 to 1
    // Positive in Index in 
    public enum IndexStates {
        STOP(0, 0),
        INDEX(0.8, 0.25),
        REVERSE(-1.0, -1.0);

        public final double indexerSpeed;
        public final double conveyorSpeed;

         IndexStates(double indexerSpeed, double conveyorSpeed) {
            this.indexerSpeed = indexerSpeed;
            this.conveyorSpeed = conveyorSpeed;
        }
    }

    // Shooter states with corresponding backspin and shooting speeds
    // Both are in Rotations Per Second (RPS) and need to set range
    // Positive for both is shoot outward 
    public enum ShooterStates {
        STOP(0, 0, 0),
        MAX(10, 0, 0),
        FORWARD_SHOOT(60, 30, 0),
        TEST1(0, 50, 0),
        TEST2( 70, 0, 0),
        VARIABLE_SHOOT(0, 0, 0),

        // SETTING THE STATES FOR DIFFERENT DISTANCES
        DISTANCE_0_5M(34,21, 0.5),
        DISTANCE_1M(32,40, 1),
        DISTANCE_1_5M(30, 40, 1.5),
        DISTANCE_2M(40,40, 2);


        
        public final double backSpinRPS;
        public final double shootingRPS;
        public final double distance; 

        ShooterStates (double backSpinRPS, double shootingRPS, double distance) {
            this.backSpinRPS = backSpinRPS;
            this.shootingRPS = shootingRPS;
            this.distance = distance; 
        }
    }


    // Intake states with corresponding intake speeds and pivot angles
    // Intake Speed range is -1 to 1 motor speed
    // Pivot is in angles need to set range
    // Positive for intake speed is intaking, positive for pivot angle is pivoting up
    public enum IntakeStates {
        STOP(0, -135),
        INTAKE(0.3, -202),
        REVERSE(-0.5,-135),
        UP(0,-135),
        DOWN(0,-135),
        MAX(1,-94),
        MIN(0,-215),
        HOME(0,-135),
        ugua(0,-170);

        public final double intakeSpeed;
        public final double pivotAngle;

        IntakeStates(double intakeSpeed, double pivotAngle) {
            this.intakeSpeed = intakeSpeed;
            this.pivotAngle = pivotAngle;
        }
    }
}
