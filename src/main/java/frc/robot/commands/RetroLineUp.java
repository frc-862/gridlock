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
        limelightFront.setPipelineNum(2); // Retro-reflective pipeline
        distance = limelightFront.getHorizontalOffset();

        Xcontroller.setSetpoint(0d);
        Xcontroller.setTolerance(AutoAlignConstants.X_TOLERANCE);

        Rcontroller.setSetpoint(90);
        Rcontroller.setTolerance(AutoAlignConstants.R_TOLERANCE);
    }

    @Override
    public void execute() {
        // If we have a target and and are on the right pipeline so that it doesn't return incorrect values 
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 2) {
            if (collector.getGamePiece() == GamePiece.CUBE) {
                Xcontroller.setSetpoint(LimelightConstants.CUBE_OFFSET);
            }

            if (drivetrain.getYaw2d().getDegrees() - 90 < AutoAlignConstants.R_TOLERANCE) { // Aligns towards the wall using pigeon heading
                distance = limelightFront.getHorizontalOffset();
                XOutput = Xcontroller.calculate(distance);
            } else { // Stops when close enough to the target angle
                XOutput = 0d;
            }
        } else {
            XOutput = 0d;
        }
        
        if (Math.abs(drivetrain.getYaw2d().getDegrees() - 90) < AutoAlignConstants.R_TOLERANCE) { // Stops when close enough to the target angle
            ROutput = 0;
        } else {
            ROutput = Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()); // Moves towards the center of the pole and stops when close enough
        }

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds( // Outputs driving 
            drivetrain.percentOutputToMetersPerSecond(XOutput),
            drivetrain.percentOutputToMetersPerSecond(0), 
            drivetrain.percentOutputToRadiansPerSecond(ROutput), 
            drivetrain.getYaw2d()));

        // Outputs values to network tables when command is running for debugging
        LightningShuffleboard.setDouble("Retro-Align", "horizontal offset", distance);
        LightningShuffleboard.setDouble("Retro-Align", "X Pid output", Xcontroller.calculate(distance));
        LightningShuffleboard.setDouble("Retro-Align", "R Pid output", Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()));
        LightningShuffleboard.setDouble("Retro-Align", "ROutput", ROutput);
        LightningShuffleboard.setDouble("Retro-Align", "XOutput", XOutput);

        Xcontroller.setP(LightningShuffleboard.getDouble("Retro-Align", "X Pee", Xcontroller.getP()));
        Xcontroller.setD(LightningShuffleboard.getDouble("Retro-Align", "X D", Xcontroller.getD()));

        Rcontroller.setP(LightningShuffleboard.getDouble("Retro-Align", "R Pee", Rcontroller.getP()));
        Rcontroller.setD(LightningShuffleboard.getDouble("Retro-Align", "R D", Rcontroller.getD()));
    }

    @Override
    public void end(boolean interrupted) {
        limelightFront.setPipelineNum(0);
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
