package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.States.IndexStates;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends Command {
    private final IndexSubsystem s_indexSubsystem;
    private IndexStates indexState;

    public IndexCommand(IndexSubsystem s_indexSubsystem) {
        this.s_indexSubsystem = s_indexSubsystem;
        addRequirements(s_indexSubsystem);
    }

    public IndexStates getIndexState() {
        return indexState;
    }

    public InstantCommand setIndexState(IndexStates indexState) {
        this.indexState = indexState;
        return new InstantCommand(() -> s_indexSubsystem.setIndexState(indexState), s_indexSubsystem);
    }
}
