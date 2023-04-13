package frc.robot.commands;

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

    public CubeAlign(Drivetrain drivetrain, LimelightFront limelightFront) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(3);
    }

    @Override
    public void execute() {
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 3){
            horizOffsetLength = limelightFront.getHorizontalOffset();
            horizOffsetLength *= AutoAlignConstants.HORIZONTAL_MULTIPLIER;
            horizOffsetDegrees = LimelightHelpers.getLimelightNTDouble("Retro", "txp");
            XOffset = horizOffsetLength / Math.sin(horizOffsetDegrees);
            YOffset = horizOffsetLength / Math.tan(horizOffsetDegrees);
        }

        translatedPose = new Pose2d(new Translation2d(drivetrain.getPose().getX() - XOffset, drivetrain.getPose().getY() - YOffset), drivetrain.getPose().getRotation());

        drivetrain.resetOdometry(translatedPose);

        LightningShuffleboard.setDouble("Cube-Align", "X offset", XOffset);
        LightningShuffleboard.setDouble("Cube-Align", "Y offset", YOffset);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal distance offset", horizOffsetLength);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal angle offset", horizOffsetLength);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
