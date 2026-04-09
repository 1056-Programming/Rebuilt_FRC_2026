import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AligntoCenter extends Command {
    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final PIDController m_pidController = new PIDController(0.1, 0, 0);

    public AligntoCenter(CommandSwerveDrivetrain driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

}