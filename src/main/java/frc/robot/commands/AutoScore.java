package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoScoreConstants.ScoreingState;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.command.core.TimedCommand;

public class AutoScore extends CommandBase {

    private Drivetrain drivetrain;
    private Collector collector;
    private Lift lift;
    private AutonomousCommandFactory autoFactory;

    private ScoreingState state = ScoreingState.setLift;

    public AutoScore(Drivetrain drivetrain, Collector collector, Lift lift, AutonomousCommandFactory autoFactory) {
        this.drivetrain = drivetrain;
        this.collector = collector;
        this.lift = lift;
        this.autoFactory = autoFactory;

        addRequirements(drivetrain, collector, lift);
    }

    @Override
    public void initialize() {
        state = ScoreingState.setLift;
    }

    @Override
    public void execute() {
        switch (state) {
            case setLift:
                lift.setGoalState(LiftState.highCubeScore);

                if (lift.onTarget()) {
                    state = ScoreingState.moveToScore;
                    drivetrain.setDesiredPose(new Pose2d(drivetrain.getPose().getX() - 1, drivetrain.getPose().getY(), drivetrain.getPose().getRotation()));
                }
                break;
            case moveToScore:
                drivetrain.moveToDesiredPose(autoFactory);
                if (drivetrain.onTarget()) {
                    state = ScoreingState.score;
                }
                break;
            case score:
                drivetrain.setDesiredPose(new Pose2d(drivetrain.getPose().getX() + 1, drivetrain.getPose().getY(), drivetrain.getPose().getRotation()));
                new TimedCommand(new InstantCommand(() -> collector.setPower(-1d)), 0.5).andThen(new InstantCommand(() -> setState(ScoreingState.moveToStow))).schedule();
                break;
            case moveToStow:
                drivetrain.moveToDesiredPose(autoFactory);
                if (drivetrain.onTarget()) {
                    state = ScoreingState.stow;
                }
                break;
            case stow:
                lift.setGoalState(LiftState.stowed);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        lift.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void setState(ScoreingState state) {
        this.state = state;
    }
}
