package frc.robot.commands.Lift;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the ground position
 */
public class Ground extends InstantCommand {
    private Lift lift;
    private Supplier<GamePiece> gamePiece;

    //read-only
    private Collector collector;

    public Ground(Lift lift, Collector collector, Supplier<GamePiece> gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;
        this.collector = collector;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (gamePiece.get() == GamePiece.CONE) {
            lift.setGoalState(LiftState.groundCone);
            collector.setGamePiece(GamePiece.CONE);
        } else {
            lift.setGoalState(LiftState.groundCube);
            collector.setGamePiece(GamePiece.CUBE);
        }
    }
}
