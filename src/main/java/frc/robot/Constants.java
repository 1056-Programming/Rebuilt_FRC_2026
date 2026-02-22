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
        public static final double kShooterP = 0.1; // kinda good P is at value 0.1
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;

        public static final double kBackP = 0;
        public static final double kBackI = 0;
        public static final double kBackD = 0;
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

    public class Vision{
        // Vision threshold: 
        public static final double ambiguityThreshold = 0.5;

        // Vision distance to camera threshold: 
        public static final double distanceThreshold = 5; 
    }

    public class CalculateShooter{
            // PHYSICAL CONSTANTS
            // PHYSICAL CONSTANTS
        public static final double GRAVITY = 32.174; // ft/s²
        public static final double AIR_DENSITY = 0.0765; // lb/ft³ at sea level
        public static final double BALL_MASS = 0.32; // lbs (typical for 8" ball)
        public static final double BALL_DIAMETER = 8.0 / 12.0; // feet (8" ball)
        public static final double BALL_CROSS_SECTION = Math.PI * Math.pow(BALL_DIAMETER / 2, 2); // ft²
        public static final double DRAG_COEFFICIENT = 0.47; // Sphere drag coefficient
        
        // MOTOR PROPERTIES
        // Launcher motor: Kraken X60 BLDC Motor WCP 1080 (022024)
        public static final double LAUNCHER_MAX_RPM = 6000; // Max RPM
        public static final double LAUNCHER_MAX_RPS = LAUNCHER_MAX_RPM / 60.0; // Max RPS
        public static final double LAUNCHER_MAX_TORQUE = 4.5; // Nm (approx)
        public static final double LAUNCHER_KV = 180; // RPM per volt (approx)
        
        // Backspin motor: Rev NEO Vortex 21-1652
        public static final double BACKSPIN_MAX_RPM = 6784; // Max RPM
        public static final double BACKSPIN_MAX_RPS = BACKSPIN_MAX_RPM / 60.0; // Max RPS
        public static final double BACKSPIN_MAX_TORQUE = 3.2; // Nm (approx)
        public static final double BACKSPIN_KV = 200; // RPM per volt (approx)
        
        // WHEEL SPECIFICATIONS
        // Launcher flywheel: 1.5 lbs, 4" diameter, 1.5" width
        public static final double LAUNCHER_WHEEL_DIAMETER = 4.0 / 12.0; // feet
        public static final double LAUNCHER_WHEEL_RADIUS = LAUNCHER_WHEEL_DIAMETER / 2;
        public static final double LAUNCHER_WHEEL_MASS = 1.5; // lbs
        public static final double LAUNCHER_WHEEL_MOMENT_OF_INERTIA = 0.5 * LAUNCHER_WHEEL_MASS * Math.pow(LAUNCHER_WHEEL_RADIUS, 2); // lb-ft²
        public static final double LAUNCHER_WHEEL_CIRCUMFERENCE = Math.PI * LAUNCHER_WHEEL_DIAMETER;
        
        // Backspin wheels: 3 wheels, each 1" wide, 2" diameter
        public static final double BACKSPIN_WHEEL_DIAMETER = 2.0 / 12.0; // feet
        public static final double BACKSPIN_WHEEL_RADIUS = BACKSPIN_WHEEL_DIAMETER / 2;
        public static final double BACKSPIN_WHEEL_CIRCUMFERENCE = Math.PI * BACKSPIN_WHEEL_DIAMETER;
        public static final int BACKSPIN_WHEEL_COUNT = 3;
        public static final double BACKSPIN_WHEEL_MASS = 0.4; // lbs per wheel (approx)
        public static final double BACKSPIN_TOTAL_INERTIA = BACKSPIN_WHEEL_COUNT * 0.5 * BACKSPIN_WHEEL_MASS * Math.pow(BACKSPIN_WHEEL_RADIUS, 2);
        
        // MECHANICAL GEOMETRY
        public static final double BACKSPIN_HORIZONTAL_OFFSET = 5.875 / 12.0; // feet
        public static final double BACKSPIN_VERTICAL_OFFSET = 4.25 / 12.0; // feet
        public static final double CONTACT_POINT_ANGLE = Math.atan2(BACKSPIN_VERTICAL_OFFSET, BACKSPIN_HORIZONTAL_OFFSET); // radians
        
        // BALL-MECHANICS INTERACTION
        public static final double FRICTION_COEFFICIENT = 0.8; // Between wheel and ball
        public static final double RESTITUTION_COEFFICIENT = 0.85; // Energy transfer efficiency
        public static final double BACKSPIN_TRANSFER_RATIO = 0.7; // How much wheel spin transfers to ball spin
        
        // SYSTEM LIMITS
        public static final double MIN_LAUNCH_VELOCITY = 10.0; // ft/s (minimum to get anywhere)
        public static final double MAX_LAUNCH_VELOCITY = 60.0; // ft/s (maximum achievable)
        public static final double MIN_ANGLE = 20.0; // degrees
        public static final double MAX_ANGLE = 80.0; // degrees

        public static final double TARGET_HEIGHT = 50.0; 
    }
}
