package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class States {
    // Indexer states with corresponding indexer and conveyor speeds
    // Both are in motor speed range of -1 to 1
    // Positive in Index in 
    public enum IndexStates {
        STOP(0, 0),
        INDEX(0.25, 0.25),
        MAX_INDEX(1,1),
        REVERSE(-0.5, -0.5);

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
    // Contains the backspin and shooterspin states for different distances
    public enum ShooterStates { 
        // Set the shooter move
        STOP(0, 0, 0),
        FORWARD_SHOOT(60, 30, 0),
        VARIABLE_SHOOT(0, 0, 0),

        // SETTING THE STATES FOR DIFFERENT DISTANCES
        IN_120(62.5,30,Units.inchesToMeters(120)),
        IN_89(32, 45,Units.inchesToMeters(89.5)),
        IN_60(22.5,37.5, Units.inchesToMeters(60)),
        IN_45(25, 35, Units.inchesToMeters(45)),
        IN_75(25, 40, Units.inchesToMeters(75)),
        IN_100(57.5, 27, Units.inchesToMeters(100));
        
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
        STOP(0, -206),

        // Set the speed of the intak-ing motor and the position of piviot 
        INTAKE(0.3, -206),
        OUTAKE(-0.5,-140),
    
        // Set the home values for piviot 
        START(0,-160),
        HOME(0,-190);

        public final double intakeSpeed;
        public final double pivotAngle;

        IntakeStates(double intakeSpeed, double pivotAngle) {
            this.intakeSpeed = intakeSpeed;
            this.pivotAngle = pivotAngle;
        }
    }
}
