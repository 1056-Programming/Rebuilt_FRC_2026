package frc.robot.subsystems;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDSubsystem extends SubsystemBase {
    private final Spark blinkin1;
    private final Spark blinkin2; // set up both blinkin
    private final Supplier<Boolean> isAtRps; // function for max. RPS for shooter motor
    private final ShooterSubsystem m_shooter;

    // Simple pattern constants (values are placeholders for Blinkin output)
    private static final double SOLID_WHITE = 0.91;
    private static final double SOLID_RED = 0.61;
    private static final double SOLID_BLUE = 0.87;
    private static final double HEARTBEAT_RED = -0.25;
    private static final double HEARTBEAT_BLUE = -0.23;

    public AddressableLEDSubsystem(int port1, int port2, Supplier<Boolean> isAtRps, ShooterSubsystem shooter) {
        this.blinkin1 = new Spark(port1);
        this.blinkin2 = new Spark(port2);
        this.isAtRps = isAtRps;
        this.m_shooter = shooter;
    }

    @Override
    public void periodic() { // called every 20 ms
        var allianceOpt = DriverStation.getAlliance(); //new code to get alliance
        double pattern = SOLID_WHITE;

        boolean atRps = false;
        if (isAtRps != null) {
            atRps = isAtRps.get();
        }

        if (atRps && alliance == DriverStation.Alliance.Red) { // Apply blinking pattern only when the RPS is max
            pattern = HEARTBEAT_RED;
        } else if (atRps && alliance == DriverStation.Alliance.Blue) { // Apply blinking pattern only when the RPS is max
            pattern = HEARTBEAT_BLUE;
        } else if (alliance == DriverStation.Alliance.Red) {
            pattern = SOLID_RED;
        } else if (alliance == DriverStation.Alliance.Blue) { // Create if-else statement for LED
            pattern = SOLID_BLUE;
        } else {
            pattern = SOLID_WHITE; // Default color if alliance is unknown
        }

        // Update outputs
        blinkin1.set(pattern);
        blinkin2.set(pattern);
    }
}                                 
