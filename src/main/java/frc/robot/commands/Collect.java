package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

/**
 * Command for running the collector with a triggers
 */
public class Collect extends CommandBase {
    Collector collector;
    DoubleSupplier leftTrigger;
    DoubleSupplier rightTrigger;

    /**
     * Creates a new Collect command
     * 
     * @param collector    the collector subsystem
     * @param leftTrigger  the left trigger
     * @param rightTrigger the right trigger
     */
    public Collect(Collector collector, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.collector = collector;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.setPower(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
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
