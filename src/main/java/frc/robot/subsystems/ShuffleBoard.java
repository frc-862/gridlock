package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalance;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

public class ShuffleBoard extends SubsystemBase {
    private LightningShuffleboardPeriodic periodicShuffleboard;
    Drivetrain drivetrain;
    Elevator elevator;
    Arm arm;
    Wrist wrist;
    Collector collector;
    AutoBalance autoBalance;
    AutoAlign autoAlign;

    public ShuffleBoard(Drivetrain drivetrain, Elevator elevator, Arm arm, Wrist wrist, Collector collector) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.collector = collector;

        initializeShuffleboard();
    }
    
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Comp Board", Constants.COMP_LOG_PERIOD, 
            new Pair<String, Object>("Elevator on Target", (BooleanSupplier) () -> elevator.onTarget()),
            new Pair<String, Object>("Arm on Target", (BooleanSupplier) () -> arm.onTarget()), 
            new Pair<String, Object>("Wrist on Target", (BooleanSupplier) () -> wrist.onTarget()),
            new Pair<String, Object>("Auto Balance on Target", (BooleanSupplier) () -> autoBalance.balanced()),
            new Pair<String, Object>("Auto Align on Target", (BooleanSupplier) () -> autoAlign.onTarget()),
            new Pair<String, Object>("Color sensor detected game piece", (Supplier<String>) () -> collector.getGamePiece().toString()));
    }

    @Override
    public void periodic() {
        periodicShuffleboard.loop();
    }
}
