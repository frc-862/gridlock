package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class Collect extends CommandBase {
    Collector collector;
    DoubleSupplier leftTrigger;
    DoubleSupplier rightTrigger;

    public Collect(Collector collector, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.collector = collector;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.runCollector(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
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
