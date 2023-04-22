package frc.robot.commands.Lift;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the high score position for cubes or cones depending on readings from
 * the color sensor
 */
public class SingleSubstationCollect extends InstantCommand {
    private Lift lift;
    private Supplier<GamePiece> gamePiece;

    public SingleSubstationCollect(Lift lift, Supplier<GamePiece> gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        // if (gamePiece.get() == GamePiece.CONE) {
        lift.setGoalState(LiftState.singleSubCone);
        // } else {
        //     lift.setGoalState(LiftState.singleSubCube);
        // }
    }
}
