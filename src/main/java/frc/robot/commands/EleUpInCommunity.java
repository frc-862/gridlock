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

    public EleUpInCommunity(Elevator elevator, Lift lift, Drivetrain drivetrain) {
        this.elevator = elevator;
        this.lift = lift;
        this.drivetrain = drivetrain;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if(lift.getCurrentState() == LiftState.stowed && lift.getGoalState() != LiftState.stowed && DriverStation.isTeleop() && currentTime - lastTime >= 1) {
            if(drivetrain.getPose().getX() < 4) {
                elevator.setExtension(4);
            } else {
                elevator.setExtension(2);
            }

            lastTime = currentTime;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
