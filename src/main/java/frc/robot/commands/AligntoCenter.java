package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain; 

public class AligntoCenter extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;

    public AligntoCenter(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {   
        m_drivetrain.getState().Pose.getX();

        double tx = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tx")
            .getDouble(0.0);

        double kP = 0.04; 
        double DegreeToTarget = tx * kP; 

     if (Math.abs(DegreeToTarget) < 1.0) {
         DegreeToTarget = 0.0; // Prevent small adjustments
         // Removed KalmanFilter/observer creation — not needed for this simple alignment command
     }
    }
}

             // Implement motor control to rotate the robot by DegreeToTarget degrees (this is a placeholder, actual implementation will depend on your robot's drivetrain)

    //private double InverseTangent(double tag_distance2, double tx) {
        // Compute angle in degrees from horizontal offset and distance.
        // This returns atan(tx / distance) in degrees; use atan2 to handle signs correctly.

