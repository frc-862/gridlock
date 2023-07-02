package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SingleSubstationAlign extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private PIDController Xcontroller = new PIDController(-0.05, 0, -0.03);
    private PIDController Rcontroller = new PIDController(0.005, 0, 0);
    private double RSetpoint = 0;
    private double XOutput = 0;
    private double ROutput = 0;
    private double distance = 0;

    /**
     * Aligns to single sub using the circle tag placed below it and the pigeon Needs alot of testing
     * 
     * @param drivetrain
     * @param limelightFront
     */
    public SingleSubstationAlign(Drivetrain drivetrain, LimelightFront limelightFront) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(3); // This pipeline is set to Circle family of AprilTags

        Xcontroller.setSetpoint(0d);
        Xcontroller.setTolerance(AutoAlignConstants.X_TOLERANCE);

        Rcontroller.setSetpoint(90);// default set based on alliance
        Rcontroller.setTolerance(AutoAlignConstants.R_TOLERANCE);

        Rcontroller.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            RSetpoint = 0;
        } else {
            RSetpoint = 180;
        }

        Rcontroller.setSetpoint(RSetpoint);
        
        // If we have a target and and are on the right pipeline so that it doesn't return incorrect values 
        if (limelightFront.hasVision() && limelightFront.getPipelineNum() == 3) { 
            if (drivetrain.getYaw2d().getDegrees() - RSetpoint < AutoAlignConstants.R_TOLERANCE) { // Moves towards the wall and stops when close enough
                distance = limelightFront.getHorizontalOffset();
                XOutput = Xcontroller.calculate(distance);
            } else {
                XOutput = 0d;
            }
        } else {
            XOutput = 0d;
        }

        if (Math.abs(drivetrain.getYaw2d().getDegrees() - RSetpoint) < AutoAlignConstants.R_TOLERANCE) { // Stops when close enough to the target angle
            ROutput = 0;
        } else {
            ROutput = Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()); // Aligns towards the wall using pigeon heading
        }

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.percentOutputToMetersPerSecond(0), 
            drivetrain.percentOutputToMetersPerSecond(XOutput),
            drivetrain.percentOutputToRadiansPerSecond(ROutput), 
            drivetrain.getYaw2d()));

        LightningShuffleboard.setDouble("sub-Align", "horizontal offset", distance);
        LightningShuffleboard.setDouble("sub-Align", "X Pid output", Xcontroller.calculate(distance));
        LightningShuffleboard.setDouble("sub-Align", "R Pid output", Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()));
        LightningShuffleboard.setDouble("sub-Align", "ROutput", ROutput);
        LightningShuffleboard.setDouble("sub-Align", "XOutput", XOutput);

        Xcontroller.setP(LightningShuffleboard.getDouble("sub-Align", "X Pee", Xcontroller.getP()));
        Xcontroller.setD(LightningShuffleboard.getDouble("sub-Align", "X D", Xcontroller.getD()));

        Rcontroller.setP(LightningShuffleboard.getDouble("sub-Align", "R Pee", Rcontroller.getP()));
        Rcontroller.setD(LightningShuffleboard.getDouble("sub-Align", "R D", Rcontroller.getD()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        limelightFront.setPipelineNum(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
