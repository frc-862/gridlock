package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.GamePiece;

/**
 * Command for running the collector with a triggers
 */
public class Collect extends CommandBase {
    Collector collector;
    DoubleSupplier input;

    /**
     * Creates a new Collect command
     * 
     * @param collector the collector subsystem
     * @param input     the input speed for the collector
     */
    public Collect(Collector collector, DoubleSupplier input) {
        this.collector = collector;
        this.input = input;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if(collector.getGamePiece() == GamePiece.CONE){
            collector.runCollector(-input.getAsDouble());
        } else {
            collector.runCollector(input.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}