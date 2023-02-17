package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

public class Stow extends SequentialCommandGroup {
    public Stow(Lift lift) {
        addCommands(
                new RunCommand(() -> lift.setNextState(LiftState.elevatorDeployed), lift)
                        .until(lift::onTarget),
                new RunCommand(() -> lift.setNextState(LiftState.stowed), lift)
                        .until(lift::onTarget));
    }
}
