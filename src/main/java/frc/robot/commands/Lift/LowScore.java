package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

public class LowScore extends InstantCommand {
    private Lift lift;

  public LowScore(Lift lift) {
    this.lift = lift;
    addRequirements(lift);
  }

  @Override
  public void initialize() {
    lift.setNextState(LiftState.ground);
  }
}
