package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.IndexStates;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.ShooterSubsytem;

public class ShooterCommand extends Command {
    private final ShooterSubsytem s_shooter;
    private IndexStates index_states;
    private ShooterStates shooter_states;

    public ShooterCommand(ShooterSubsytem shooter, IndexStates iState, ShooterStates sState) {
        this.s_shooter = shooter;
        this.index_states = iState;
        this.shooter_states = sState;

        addRequirements(s_shooter);
    }

    @Override
    public void initialize() {
        s_shooter.setShootingState(shooter_states);
        s_shooter.setIndexState(index_states);
    }

}