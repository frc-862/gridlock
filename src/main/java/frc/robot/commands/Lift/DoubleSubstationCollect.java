package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

public class DoubleSubstationCollect extends InstantCommand {
    private Lift lift;

    public DoubleSubstationCollect(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setNextState(LiftState.doubleSubstationCollect);
    }
}
