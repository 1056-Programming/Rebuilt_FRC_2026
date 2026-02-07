package frc.robot;

public class States {
    public enum IndexStates {
        STOP(0, 0),
        INDEX(1.0, 1.0),
        REVERSE(-1.0, -1.0);

        public final double indexerSpeed;
        public final double conveyorSpeed;

         IndexStates(double indexerSpeed, double conveyorSpeed) {
            this.indexerSpeed = indexerSpeed;
            this.conveyorSpeed = conveyorSpeed;
        }
    }

    public enum ShooterStates {
        STOP(0, 0),
        SHOOT(1, 1),
        FORWARD_SHOOT(-1, -1),
        VARIABLE_SHOOT(0, 0);
        
        public final double backSpinSpeed;
        public final double shootingSpeed;

        ShooterStates (double backSpinSpeed, double shootingSpeed) {
            this.backSpinSpeed = backSpinSpeed;
            this.shootingSpeed = shootingSpeed;
        }
    }

    public enum IntakeStates {
        STOP(0, 0),
        INDEX(1.0, 1.0),
        REVERSE(-1.0, -1.0);

        public final double indexerSpeed;
        public final double conveyorSpeed;

         IntakeStates(double indexerSpeed, double conveyorSpeed) {
            this.indexerSpeed = indexerSpeed;
            this.conveyorSpeed = conveyorSpeed;
        }
    }
}
