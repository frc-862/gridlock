package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AutoAlign extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelight;
    private PIDController controller;

    public AutoAlign(Drivetrain drivetrain, LimelightFront limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        controller = new PIDController(AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kP, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kI, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kD);
        controller.setSetpoint(AutoAlignConstants.OFFSET);
        controller.setTolerance(AutoAlignConstants.TOLERANCE);
        
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipelineNum(2);
    }

    @Override
    public void execute() {
        LightningShuffleboard.setBool("Auto align", "OnTarget", isOnTarget(limelight.getHorizontalOffset()));
        LightningShuffleboard.setDouble("Auto align", "Horizontal offset", limelight.getHorizontalOffset());

        if (limelight.hasVision()) {
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    drivetrain.percentOutputToMetersPerSecond(
                            controller.calculate(limelight.getHorizontalOffset())),
                    drivetrain.percentOutputToMetersPerSecond(0d),
                    drivetrain.percentOutputToRadiansPerSecond(0d),
                    drivetrain.getYaw2d()));
        } else {
            drivetrain.stop();
        }
    }

    public boolean isOnTarget(double currentAngle) {
        currentAngle -= AutoAlignConstants.OFFSET;
        return Math.abs(currentAngle) < AutoAlignConstants.TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return isOnTarget(limelight.getHorizontalOffset());
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setPipelineNum(0);
        drivetrain.stop();
    }
}
