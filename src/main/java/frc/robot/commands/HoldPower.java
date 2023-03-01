package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.GamePiece;

public class HoldPower extends CommandBase {
    private Collector collector;

    public HoldPower(Collector collector) {
        this.collector = collector;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (collector.getGamePiece().equals(GamePiece.CONE) || collector.getGamePiece().equals(GamePiece.CUBE)) {
            collector.setPower(CollectorConstants.HOLD_POWER);
        } else {
            collector.setPower(0d);
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
