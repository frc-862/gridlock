package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the high score position for cubes or cones depending on readings from
 * the color sensor
 */
public class MidScore extends InstantCommand {
    private Lift lift;
    private GamePiece gamePiece;

    public MidScore(Lift lift, GamePiece gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (gamePiece == GamePiece.CONE) {
            lift.setGoalState(LiftState.midConeScore);
        } else {
            lift.setGoalState(LiftState.midCubeScore);
        }
    }
}
