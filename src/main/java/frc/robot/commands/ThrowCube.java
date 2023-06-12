package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lift;

public class ThrowCube extends CommandBase {
    private Arm arm;
    private Lift lift;
    private Collector collector;

    public ThrowCube(Lift lift, Arm arm, Collector collector) {
        this.arm = arm;
        this.lift = lift;
        this.collector = collector;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        lift.setGoalState(LiftState.OTB_High);
        collector.setPower(1);
    }

    @Override
    public void execute() {
        if (arm.getAngle().getDegrees() >= 110) {
            collector.setPower(-1);
        } else {
            collector.setPower(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return lift.onTarget();
    }
}
