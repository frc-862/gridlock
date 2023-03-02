package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the ground position
 */
public class Ground extends InstantCommand {
    private Lift lift;
    private GamePiece gamePiece;

    public Ground(Lift lift, GamePiece gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (gamePiece == GamePiece.CONE) {
            lift.setGoalState(LiftState.groundCone);
        } else {
            lift.setGoalState(LiftState.groundCube);
        }
    }
}
