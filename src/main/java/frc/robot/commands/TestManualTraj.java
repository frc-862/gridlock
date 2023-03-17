package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;

public class TestManualTraj extends SequentialCommandGroup {
    public TestManualTraj(Drivetrain drivetrain, AutonomousCommandFactory autoFactory) {
        addCommands(
                new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())),
                autoFactory.createManualTrajectory(new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION), drivetrain.getCurrentPathPoint(),
                        new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(0))));
    }
}
