package frc.robot.commands.Lift;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Sets the lift position to the ground position
 */
public class GroundSequential extends SequentialCommandGroup {

    public GroundSequential(Lift lift, Collector collector, Wrist wrist, Supplier<GamePiece> gamePiece) {
        if (gamePiece.get() == GamePiece.CONE) {
            addCommands(new RunCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(112))).until(wrist::onTarget), new InstantCommand(() -> lift.setGoalState(LiftState.groundCone)),
                    new InstantCommand(() -> collector.setGamePiece(GamePiece.CONE)));
        } else {
            addCommands(new RunCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(112))).until(wrist::onTarget), new InstantCommand(() -> lift.setGoalState(LiftState.groundCube)),
                    new InstantCommand(() -> collector.setGamePiece(GamePiece.CUBE)));
        }
        addRequirements(lift, wrist);

    }
}
