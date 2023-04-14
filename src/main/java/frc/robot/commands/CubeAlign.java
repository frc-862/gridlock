package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.swervelib.DriveController;

public class CubeAlign extends CommandBase {

    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private double horizOffsetLength;
    private double horizOffsetDegrees;
    private Pose2d translatedPose = new Pose2d();
    private double XOffset = 0;
    private double YOffset = 0;
    private PIDController xController = new PIDController(0.001, 0, 0);
    private PIDController yController = new PIDController(0.001, 0, 0);

    public CubeAlign(Drivetrain drivetrain, LimelightFront limelightFront) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;

        xController.setTolerance(3);
        yController.setTolerance(3);

    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(3);
    }

    @Override
    public void execute() {
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 3) {
            horizOffsetLength = limelightFront.getHorizontalOffset();
            horizOffsetLength *= AutoAlignConstants.HORIZONTAL_MULTIPLIER;
            horizOffsetDegrees = LimelightHelpers.getLimelightNTDouble("Retro", "tx");

            if (Math.sin(Math.toRadians(horizOffsetDegrees)) != 0 && Math.tan(Math.toRadians(horizOffsetDegrees)) != 0) {
                XOffset = horizOffsetLength / Math.sin(Math.toRadians(horizOffsetDegrees));
                YOffset = horizOffsetLength / Math.tan(Math.toRadians(horizOffsetDegrees));

            } else {
                XOffset = 0;
                YOffset = 0;
            }

        }

        double xPoseOffset = xController.calculate(drivetrain.getPose().getX(), drivetrain.getPose().getX() - XOffset);
        double yPoseOffset = yController.calculate(drivetrain.getPose().getY(), drivetrain.getPose().getY() - XOffset);

        translatedPose = drivetrain.getPose().transformBy(new Transform2d(new Translation2d(-xPoseOffset, -yPoseOffset), Rotation2d.fromDegrees(0)));

        // drivetrain.resetOdometry(translatedPose);

        LightningShuffleboard.setDouble("Cube-Align", "X offset", XOffset);
        LightningShuffleboard.setDouble("Cube-Align", "Y offset", YOffset);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal distance offset", horizOffsetLength);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal angle offset", horizOffsetDegrees);
        LightningShuffleboard.setDouble("Cube-Align", "X pose offset", xPoseOffset);
        LightningShuffleboard.setDouble("Cube-Align", "Y pose offset", yPoseOffset);
        LightningShuffleboard.setDouble("Cube-Align", "X pose", translatedPose.getX());
        LightningShuffleboard.setDouble("Cube-Align", "Y pose", translatedPose.getY());
        LightningShuffleboard.setDouble("Cube-Align", "rotation pose", translatedPose.getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        limelightFront.setPipelineNum(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
