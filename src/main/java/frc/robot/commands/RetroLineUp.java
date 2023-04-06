package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Collector.GamePiece;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class RetroLineUp extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private PIDController Xcontroller = new PIDController(-0.05, 0, -0.03);
    private PIDController Rcontroller = new PIDController(0.005, 0, 0);
    private Collector collector;
    private double XOutput = 0;
    private double ROutput = 0;
    private double distance = 0;

    public RetroLineUp(Drivetrain drivetrain, LimelightFront limelightFront, Collector collector) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;
        this.collector = collector;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(2); // TODO check if this is the right pipeline
        distance = limelightFront.getHorizontalOffset();

        Xcontroller.setSetpoint(0d);
        Xcontroller.setTolerance(AutoAlignConstants.X_TOLERANCE);

        Rcontroller.setSetpoint(90);
        Rcontroller.setTolerance(AutoAlignConstants.R_TOLERANCE);
    }

    @Override
    public void execute() {
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 2) {
            if (collector.getGamePiece() == GamePiece.CUBE) {
                //TODO figure out the difference between the two on the x to change the offset
                Xcontroller.setSetpoint(LimelightConstants.CUBE_OFFSET);
            }

            if (drivetrain.getYaw2d().getDegrees() < AutoAlignConstants.R_TOLERANCE) {
                distance = limelightFront.getHorizontalOffset();
                XOutput = Xcontroller.calculate(distance);
            } else {
                XOutput = 0d;
            }
        } else {
            XOutput = 0d;
        }
        
        if (drivetrain.getYaw2d().getDegrees() < AutoAlignConstants.R_TOLERANCE) {
            ROutput = 0;
        } else {
            ROutput = Rcontroller.calculate(drivetrain.getYaw2d().getDegrees());
        }

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.percentOutputToMetersPerSecond(XOutput),
            drivetrain.percentOutputToMetersPerSecond(0), 
            drivetrain.percentOutputToRadiansPerSecond(ROutput), 
            drivetrain.getYaw2d()));


        LightningShuffleboard.setDouble("Retro-Align", "horizontal offset", distance);
        LightningShuffleboard.setDouble("Retro-Align", "X Pid output", Xcontroller.calculate(distance));
        LightningShuffleboard.setDouble("Retro-Align", "R Pid output", Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()));
        LightningShuffleboard.setDouble("Retro-Align", "ROutput", ROutput);
        LightningShuffleboard.setDouble("Retro-Align", "XOutput", XOutput);
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
