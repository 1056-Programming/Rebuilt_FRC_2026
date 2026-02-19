package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private final IntakeSubsystem s_intakesubsystem;
    private IntakeStates intakeState;

    public IntakeCommand(IntakeSubsystem s_intakesubsystem) {
        this.s_intakesubsystem = s_intakesubsystem;
        addRequirements(s_intakesubsystem);
    }

    // return current set intake state
    public IntakeStates getIntakeState() {
        return intakeState;
    }

    // return command to set intake state 
    public InstantCommand setIntakeState(IntakeStates intakeState) {
        this.intakeState = intakeState;
        return new InstantCommand(() -> s_intakesubsystem.setIntakeState(intakeState), s_intakesubsystem);
    }
}
