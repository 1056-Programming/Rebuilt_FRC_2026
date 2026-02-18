package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//Ensuring shooting is correct and connect to the Xbox
public class Shooter extends Command {
    private final ShooterSubsystem m_shootersubsystem;
    private final XboxController m_controller;

    public ShooterCommand(ShooterSubsystem m_ShooterSubsystem) {
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        addRequirements(m_ShooterSubsystem);


    }

}
}