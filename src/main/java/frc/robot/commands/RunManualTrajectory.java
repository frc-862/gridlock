package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.thunder.auto.AutonomousCommandFactory;

public class RunManualTrajectory extends CommandBase {
    private Drivetrain drivetrain;
    private LEDs leds;
    private Collector collector;
    private AutonomousCommandFactory autoFactory;

    private Pose2d relativePsoe;

    public RunManualTrajectory(Drivetrain drivetrain, LEDs leds, Collector collector, AutonomousCommandFactory autoFactory) {
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.collector = collector;
        this.autoFactory = autoFactory;

        addRequirements(drivetrain, leds);
    }

    @Override
    public void initialize() {
        drivetrain.moveToDesiredPose(autoFactory);
        leds.wantsPiece(collector.getGamePiece());
        }

    @Override
    public void execute() {
        relativePsoe = drivetrain.getPose().relativeTo(drivetrain.getDesiredPose());
    }

    @Override
    public void end(boolean interrupted) {
        leds.isAligned(collector.getGamePiece());
    }

    @Override
    public boolean isFinished() {
        return relativePsoe.getX() < 0.1 && relativePsoe.getY() < 0.1 && relativePsoe.getRotation().getDegrees() < 1;
    }
}
