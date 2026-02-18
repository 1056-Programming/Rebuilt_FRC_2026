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
        
        public final double backSpinRPS;
        public final double shootingRPS;

        ShooterStates (double backSpinRPS, double shootingRPS) {
            this.backSpinRPS = backSpinRPS;
            this.shootingRPS = shootingRPS;
        }
    }

    public enum IntakeStates {
        STOP(0, 0),
        INDEX(1.0, 1.0),
        REVERSE(-1.0, -1.0);

        public final double intakeSpeed;
        public final double angle;

        IntakeStates(double intakeSpeed, double angle) {
            this.intakeSpeed = intakeSpeed;
            this.angle = angle;
        }
    }
}
