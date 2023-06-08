package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Maps;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.ServoTurn;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class CubeAlign extends CommandBase {

    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private ServoTurn servoTurn;
    private Lift lift;
    private LEDs leds;
    private Arm arm;
    private Collector collector;

    private double horizOffsetLength;
    private double horizOffsetDegrees;
    private Pose2d translatedPose = new Pose2d();
    private double XOffset = 0;
    private double YOffset = 0;
    private PIDController xController = new PIDController(0.001, 0, 0);
    private PIDController yController = new PIDController(0.001, 0, 0);
    private boolean spiked = false;
    private int cycle = -1;
    private double startTime = 0;

    private final AutonomousCommandFactory autoFactory;

    public CubeAlign(Drivetrain drivetrain, LimelightFront limelightFront, ServoTurn servoTurn, Lift lift, LEDs leds, Arm arm, Collector collector, int cycle) {
        this.drivetrain = drivetrain;
        this.limelightFront = limelightFront;
        this.servoTurn = servoTurn;
        this.lift = lift;
        this.leds = leds;
        this.arm = arm;
        this.collector = collector;
        this.cycle = cycle;

        xController.setTolerance(3);
        yController.setTolerance(3);

        autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(), AutonomousConstants.DRIVE_PID_CONSTANTS,
                AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelightFront.setPipelineNum(3);
        startTime = Timer.getFPGATimestamp();
        xController.setSetpoint(0);
        yController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (collector.isStalling()) {
            spiked = true;
        }

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

        

        // // THIS is for pose correction could be used while a drive command is running 
        // double xPoseOffset = xController.calculate(drivetrain.getPose().getX(), drivetrain.getPose().getX() - XOffset);
        // double yPoseOffset = yController.calculate(drivetrain.getPose().getY(), drivetrain.getPose().getY() - YOffset);

        // translatedPose = drivetrain.getPose().transformBy(new Transform2d(new Translation2d(-xPoseOffset, -yPoseOffset), Rotation2d.fromDegrees(0)));
        
        // THIS is direct control over the drivetrain and can not be used in tandem
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        drivetrain.percentOutputToMetersPerSecond(xController.calculate(XOffset)), 
        drivetrain.percentOutputToMetersPerSecond(yController.calculate(YOffset)),
        drivetrain.percentOutputToRadiansPerSecond(0), 
        drivetrain.getYaw2d()));

        LightningShuffleboard.setDouble("Cube-Align", "X offset", XOffset);
        LightningShuffleboard.setDouble("Cube-Align", "Y offset", YOffset);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal distance offset", horizOffsetLength);
        LightningShuffleboard.setDouble("Cube-Align", "horizontal angle offset", horizOffsetDegrees);
        // LightningShuffleboard.setDouble("Cube-Align", "X pose offset", xPoseOffset);
        // LightningShuffleboard.setDouble("Cube-Align", "Y pose offset", yPoseOffset);
        LightningShuffleboard.setDouble("Cube-Align", "X pose", translatedPose.getX());
        LightningShuffleboard.setDouble("Cube-Align", "Y pose", translatedPose.getY());
        LightningShuffleboard.setDouble("Cube-Align", "rotation pose", translatedPose.getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        limelightFront.setPipelineNum(0);
        if (DriverStation.isAutonomous()) {
            if (cycle == 1) {
                autoFactory.makeTrajectory("CUBE-TEST-2", Maps.getPathMap(drivetrain, servoTurn, lift, collector, leds, arm, limelightFront, 2), new PathConstraints(3.5, 2));
            } else if (cycle == 2) {
                autoFactory.makeTrajectory("CUBE-TEST-3", Maps.getPathMap(drivetrain, servoTurn, lift, collector, leds, arm, limelightFront, 3), new PathConstraints(3.5, 2));
            }
        }
    }

    @Override
    public boolean isFinished() {
        if ((spiked && !limelightFront.hasVision()) || Timer.getFPGATimestamp() - startTime > 5) {
            return true;
        }
        return false;
    }
}
