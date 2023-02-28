package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Collector.GamePiece;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AutoAlign extends CommandBase {
    private Drivetrain drivetrain;
    private Limelight limelight;
    private GamePiece pieceType;
    private PIDController controller = new PIDController(0.002, 0, 0);
    public AutoAlign(Drivetrain drivetrain, Limelight limelight, GamePiece pieceType) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.pieceType = pieceType;

        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        switch (pieceType) {
            case CONE:
                limelight.setPipelineNum(0);
                break;
            case CUBE:
                limelight.setPipelineNum(1);
                break;
            default:
                limelight.setPipelineNum(0);
                break;
        }
        controller.setSetpoint(AutoAlignConstants.OFFSET);
        controller.setTolerance(AutoAlignConstants.TOLERANCE);
    }

    @Override
    public void execute() {
        controller.setP(LightningShuffleboard.getDouble("Auto align", "kP", controller.getP()));
        controller.setI(LightningShuffleboard.getDouble("Auto align", "kI", controller.getI()));
        controller.setD(LightningShuffleboard.getDouble("Auto align", "kD", controller.getD()));


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
        drivetrain.stop();
    }
}
