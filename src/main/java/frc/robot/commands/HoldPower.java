package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.GamePiece;

public class HoldPower extends CommandBase {
    Collector collector;
    DoubleSupplier input;
    boolean doHoldPower = false;
    double power = 0;
    XboxController driver;
    XboxController copilot;

    /**
     * Creates a new Collect command
     * 
     * @param collector the collector subsystem
     * @param input the input speed for the collector
     */
    public HoldPower(Collector collector, DoubleSupplier input, XboxController driver, XboxController copilot ) {
        this.collector = collector;
        this.input = input;
        this.driver = driver;
        this.copilot = copilot;
        
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
            if(collector.getGamePiece() == GamePiece.CUBE){
                power = CollectorConstants.HOLD_POWER_CUBE;
            } else{
                power = CollectorConstants.HOLD_POWER_CONE;
            }

        } else {
            power = 0;
        }

        if (input.getAsDouble() < 0) {
            collector.setCurrentLimit(60);
        } else {
            collector.setCurrentLimit(CollectorConstants.CURRENT_LIMIT);

        }

        if(DriverStation.isTeleop()) {
            collector.setPower(power);

            if(collector.isStalling()) {
                driver.setRumble(RumbleType.kBothRumble, 1);
                copilot.setRumble(RumbleType.kBothRumble, 1);
            } else {
                driver.setRumble(RumbleType.kBothRumble, 0);
                copilot.setRumble(RumbleType.kBothRumble, 0);
            }
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
