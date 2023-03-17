package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

public class AutoAlign extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelight;
    private PIDController controller;

    private double OFFSET;

    public AutoAlign(Drivetrain drivetrain, LimelightFront limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        if (Constants.isBlackout()) {
            // If blackout, use the blackout offset
            OFFSET = AutoAlignConstants.LIMELGHT_OFFSET_BLACKOUT;
        } else {
            // Otherwise, assume gridlock offset
            OFFSET = AutoAlignConstants.LIMELGHT_OFFSET_GRIDLOCK;
        }
        
        controller = new PIDController(AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kP, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kI, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kD);
        controller.setSetpoint(OFFSET);
        controller.setTolerance(AutoAlignConstants.TOLERANCE);

        // Initialize the shuffleboard values and start logging data
        initialiizeShuffleboard();

        addRequirements(drivetrain, limelight);
    }
        // Periodic Shuffleboard
        private LightningShuffleboardPeriodic periodicShuffleboard;

    @Override
    public void initialize() {
        limelight.setPipelineNum(2);
    }

    // logs auto align things
    @SuppressWarnings("unchecked")
    private void initialiizeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Auto align", AutoAlignConstants.LOG_PERIOD,
                new Pair<String, Object>("Auto align On Target", (BooleanSupplier) () -> onTarget()));;
                new Pair<String, Object>("Horizontal Offset", (DoubleSupplier) () -> limelight.getHorizontalOffset() - OFFSET);;

    }
    @Override
    public void execute() {

        //LightningShuffleboard.setBool("Auto align", "OnTarget",  onTarget());
        //LightningShuffleboard.setDouble("Auto align", "Horizontal offset", limelight.getHorizontalOffset() - OFFSET);

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

    public boolean onTarget() {
        double currentAngle = limelight.getHorizontalOffset();
        currentAngle -= OFFSET;
        return Math.abs(currentAngle) < AutoAlignConstants.TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setPipelineNum(0);
        drivetrain.stop();
    }
}
