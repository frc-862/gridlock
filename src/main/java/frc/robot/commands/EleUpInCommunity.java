package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;

public class EleUpInCommunity extends CommandBase {
    private Elevator elevator;
    private Lift lift;
    private Drivetrain drivetrain;
    private double lastTime = 0;
    private double currentTime = -1;

    public EleUpInCommunity(Elevator elevator, Lift lift, Drivetrain drivetrain) {
        this.elevator = elevator;
        this.lift = lift;
        this.drivetrain = drivetrain;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if(lift.getCurrentState() == LiftState.stowed && lift.getGoalState() == LiftState.stowed && DriverStation.isTeleop() && currentTime - lastTime >= 1) {
            if(drivetrain.isInCommunity()) {
                lift.setGoalState(LiftState.elevatorDeployed);
            } else {
                lift.setGoalState(LiftState.stowed);
            }

            lastTime = currentTime;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
