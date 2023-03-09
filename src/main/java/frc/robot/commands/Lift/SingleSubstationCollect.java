package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

/**
 * Sets the lift position to the reverse double substation collect position
 */
public class SingleSubstationCollect extends InstantCommand {
    private Lift lift;

    public SingleSubstationCollect(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setGoalState(LiftState.singleSubstationCollect);
    }
}
