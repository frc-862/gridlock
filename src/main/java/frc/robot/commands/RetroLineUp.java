package frc.robot.commands;

import org.opencv.core.TickMeter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Collector.GamePiece;

public class RetroLineUp extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private PIDController controller = new PIDController(0.009, 0, 0);
    private Collector collector;

    private double distance;

    public RetroLineUp(Drivetrain drivetrain, LimelightFront limelightFront, Collector collector) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;
        this.collector = collector;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(1); // TODO check if this is the right pipeline
        distance = limelightFront.getHorizontalOffset();

        controller.setSetpoint(LimelightConstants.FRONT_POSE.getX());
        controller.setTolerance(AutoAlignConstants.TOLERANCE);
    }

    @Override
    public void execute() {
        if(limelightFront.hasVision()){
            if(collector.getGamePiece() == GamePiece.CUBE){
                //TODO figure out the difference between the two on the x to change the offset
                controller.setSetpoint(LimelightConstants.FRONT_POSE.getX() + LimelightConstants.CUBE_OFFSET);
            } else {
                distance = limelightFront.getHorizontalOffset();
            }

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.percentOutputToMetersPerSecond(controller.calculate(distance)),
            drivetrain.percentOutputToMetersPerSecond(0),
            drivetrain.percentOutputToRadiansPerSecond(0),
            drivetrain.getYaw2d()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelightFront.setPipelineNum(0);
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
