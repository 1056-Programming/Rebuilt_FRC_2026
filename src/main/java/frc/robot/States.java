package frc.robot;

public class States {
    public enum ShooterStates {
        STOP(0, 0),
        SHOOT(1, 1),
        FORWARD_SHOOT(-1, -1);
        
        public final double backSpinSpeed;
        public final double shootingSpeed;

        ShooterStates (double backSpinSpeed, double shootingSpeed) {
            this.backSpinSpeed = backSpinSpeed;
            this.shootingSpeed = shootingSpeed;
        }
    }

    public enum IndexStates {
        STOP(0, 0),
        SHOOT(1, 1);

        public final double indexerSpeed;
        public final double conveyorSpeed;

        IndexStates (double indexerSpeed, double conveyorSpeed) {
            this.indexerSpeed = indexerSpeed;
            this.conveyorSpeed = conveyorSpeed;
        }
    }

}
