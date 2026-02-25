package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem s_ShooterSubsystem; 
    private ShooterStates shooterStates;
    
    public ShooterCommand(ShooterSubsystem s_ShooterSubsystem) {
        this.s_ShooterSubsystem = s_ShooterSubsystem; 
        addRequirements(s_ShooterSubsystem);
    }

    public ShooterStates getShooterStates() {
        return shooterStates; 
    }

    public InstantCommand setShooterState(ShooterStates shooterStates) {
        this.shooterStates = shooterStates; 

        return new InstantCommand(() -> s_ShooterSubsystem.setShooterState(shooterStates),s_ShooterSubsystem); 
    }
}
