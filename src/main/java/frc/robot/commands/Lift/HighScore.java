package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

public class HighScore extends InstantCommand {
    private Lift lift;
    private boolean isCone;

    public HighScore(Lift lift, boolean isCone) {
        this.lift = lift;
        this.isCone = isCone;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (isCone) {
            lift.setGoalState(LiftState.highConeScore);
        } else {
            lift.setGoalState(LiftState.highCubeScore);
        }
    }
}
