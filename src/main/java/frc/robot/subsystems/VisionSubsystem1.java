package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem1 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem1(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

     @Override
     public void periodic() {
         // This method will be called once per scheduler run
     }

     
}
