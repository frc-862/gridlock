package frc.robot.commands.Lift;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the ground position
 */
public class Ground extends InstantCommand {
    private Lift lift;
    private Supplier<GamePiece> gamePiece;

    public Ground(Lift lift, Supplier<GamePiece> gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (gamePiece.get() == GamePiece.CONE) {
            lift.setGoalState(LiftState.groundCone);
        } else {
            lift.setGoalState(LiftState.groundCube);
        }
    }
}
