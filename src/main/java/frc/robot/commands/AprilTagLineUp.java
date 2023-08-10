package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AprilTagLineUp extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private PIDController Xcontroller = new PIDController(-0.05, 0, -0.03);
    private PIDController Rcontroller = new PIDController(0.005, 0, 0);
    private double RSetpoint = 0;
    private double XOutput = 0;
    private double ROutput = 90;
    private double distance = 0;

    public AprilTagLineUp(Drivetrain drivetrain, LimelightFront limelightFront) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(0); // Uses fornt limelight pipeline that is able to see all field AprilTags 

        Xcontroller.setSetpoint(0d);
        Xcontroller.setTolerance(AutoAlignConstants.X_TOLERANCE);

        Rcontroller.setSetpoint(RSetpoint);
        Rcontroller.setTolerance(AutoAlignConstants.R_TOLERANCE);

        Rcontroller.enableContinuousInput(0, 360);
    }

    //90 is 180

    @Override
    public void execute() {
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

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.percentOutputToMetersPerSecond(XOutput), 
        drivetrain.percentOutputToMetersPerSecond(0),
        drivetrain.percentOutputToRadiansPerSecond(ROutput), 
        drivetrain.getYaw2d()));

        // Only runs while button is pressed For Testing and performance evaluation
        LightningShuffleboard.setDouble("April-Align", "horizontal offset", distance);
        LightningShuffleboard.setDouble("April-Align", "X Pid output", Xcontroller.calculate(distance));
        LightningShuffleboard.setDouble("April-Align", "R Pid output", Rcontroller.calculate(drivetrain.getYaw2d().getDegrees()));
        LightningShuffleboard.setDouble("April-Align", "ROutput", ROutput);
        LightningShuffleboard.setDouble("April-Align", "XOutput", XOutput);

        Xcontroller.setP(LightningShuffleboard.getDouble("April-Align", "X Pee", Xcontroller.getP()));
        Xcontroller.setD(LightningShuffleboard.getDouble("April-Align", "X D", Xcontroller.getD()));

        Rcontroller.setP(LightningShuffleboard.getDouble("April-Align", "R Pee", Rcontroller.getP()));
        Rcontroller.setD(LightningShuffleboard.getDouble("April-Align", "R D", Rcontroller.getD()));
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
