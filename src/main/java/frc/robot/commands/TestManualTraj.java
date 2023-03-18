package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;

public class TestManualTraj extends CommandBase {

    private Drivetrain drivetrain;
    private AutonomousCommandFactory autoFactory;
    private BooleanSupplier buttonPress;

    public TestManualTraj(Drivetrain drivetrain, AutonomousCommandFactory autoFactory, BooleanSupplier buttonPress) {
        this.drivetrain = drivetrain;
        this.autoFactory = autoFactory;
        this.buttonPress = buttonPress;
    }

    @Override
    public void initialize() {
        autoFactory.createManualTrajectory(new PathConstraints(0.1, 0.1), drivetrain.getPose(), new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return buttonPress.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
