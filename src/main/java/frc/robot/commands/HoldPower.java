package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class HoldPower extends CommandBase {
    Collector collector;
    DoubleSupplier input;
    boolean doHoldPower = false;
    double power = 0;

    /**
     * Creates a new Collect command
     * 
     * @param collector the collector subsystem
     * @param input the input speed for the collector
     */
    public HoldPower(Collector collector, DoubleSupplier input) {
        this.collector = collector;
        this.input = input;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (input.getAsDouble() > 0) {
            doHoldPower = true;
            power = input.getAsDouble();
        } else if (input.getAsDouble() < 0) {
            doHoldPower = false;
            power = input.getAsDouble();
        } else if (doHoldPower) {
            power = CollectorConstants.HOLD_POWER;
        } else {
            power = 0;
        }

        collector.setPower(power);
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
