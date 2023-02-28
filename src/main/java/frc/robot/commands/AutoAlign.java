
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AutoAlign extends CommandBase {
    private Drivetrain drivetrain;
    private Vision vision;
    private int pipeline = 0;
    private double offset = -4.5;
    private PIDController controller = new PIDController(0.002, 0, 0);

    public AutoAlign(Drivetrain drivetrain, Vision vision, int pipeline) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.pipeline = pipeline;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        vision.setPipelineNum(pipeline);
        controller.setSetpoint(offset);
        controller.setTolerance(3d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!isOnTarget(vision.getHorizontalOffset()) && vision.getHasVision()) {
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(

                    drivetrain.percentOutputToMetersPerSecond(
                            controller.calculate(vision.getHorizontalOffset())),
                    drivetrain.percentOutputToMetersPerSecond(0d),
                    drivetrain.percentOutputToRadiansPerSecond(0d), drivetrain.getYaw2d()));
        } else {
            drivetrain.stop();
        }
    }

    public boolean isOnTarget(double currentAngle) {
        currentAngle -= 4.5;
        // Should put consideration into how accurate we want to be later on.
        return Math.abs(currentAngle) < 3;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        vision.setPipelineNum(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {;
    }

}
