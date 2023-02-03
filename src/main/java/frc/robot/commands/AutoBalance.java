package frc.robot.commands;

import java.util.concurrent.PriorityBlockingQueue;
import javax.swing.ToolTipManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.filter.MovingAverageFilter;
import frc.thunder.math.LightningMath;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;

    private double lastTime = 0;
    private double pitchAngle;
    private double rollAngle;
    private double theta;
    private double magnitude;
    private double speedMetersPerSecond;
    private double magnitudeRateOfChange;
    private double lastMagnitude;

    private MovingAverageFilter filter;

    private SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()};


    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        filter = new MovingAverageFilter(5);

        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {}


    @Override
    public void execute() {

        // magnitudeRateOfChange = lastMagnitude - magnitude;

        // filter.filter(magnitudeRateOfChange);

        if (Timer.getFPGATimestamp() - lastTime > AutoBalanceConstants.THRESHOLD_TIME) {
            lastTime = Timer.getFPGATimestamp();

            pitchAngle = drivetrain.getPitch2d().getDegrees();
            rollAngle = drivetrain.getRoll2d().getDegrees();
            theta = LightningMath.inputModulus(-(Math.atan2(rollAngle, pitchAngle) + Math.PI),
                    -Math.PI, Math.PI);
            magnitude = Math.sqrt((pitchAngle * pitchAngle) + (rollAngle * rollAngle));

            speedMetersPerSecond =
                    MathUtil.clamp(magnitude * AutoBalanceConstants.MAGNITUDE_SCALER, -0.6, 0.6);

            for (int i = 0; i < moduleStates.length; i++) {
                moduleStates[i] =
                        new SwerveModuleState(speedMetersPerSecond, new Rotation2d(theta));

            }

            if (magnitude > AutoBalanceConstants.MAGNITUDE_THRESHOLD) {
                drivetrain.setStates(moduleStates);
            } else {
                drivetrain.stop();
            }
        }
        // totalAngle = Math.abs(pitchAngle) + Math.abs(rollAngle);

        // if (pitchAngle >= 0) {
        // if (rollAngle >= 0) {
        // offset = 315;
        // } else {
        // offset = 45;
        // }
        // } else {
        // if (rollAngle >= 0) {
        // offset = 225;
        // } else {
        // offset = 135;
        // }
        // }
        // finalAngle = offset + 45 * (pitchAngle / totalAngle - rollAngle / totalAngle);

        // averageAngle = totalAngle / 2;

        // angleDelta = lastAverageAngle - averageAngle;

        // lastAverageAngle = averageAngle;

        // LightningShuffleboard.setDouble("auto balance", "final angle", finalAngle);
        // }

        // if (averageAngle > AutoBalanceConstants.AVERAGE_ANGLE_THRESHOLD) {
        // for (int i = 0; i < moduleStates.length; i++) {
        // moduleStates[i] = new SwerveModuleState(speed, new Rotation2d(finalAngle));
        // }

        // drivetrain.setStates(moduleStates);

        // } else if (angleDelta < AutoBalanceConstants.ANGLE_DELTA_THRESHOLD) {
        // drivetrain.stop();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
