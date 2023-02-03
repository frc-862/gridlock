package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.filter.MovingAverageFilter;
import frc.thunder.logging.DataLogger;
import frc.thunder.math.LightningMath;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;

    private double pitchAngle;
    private double rollAngle;
    private double theta;
    private double magnitude;
    private double speedMetersPerSecond;
    private double magnitudeRateOfChange;
    private double filteredMagnitudeRateOfChange;
    private double lastMagnitude;
    
    private Debouncer delay = new Debouncer(AutoBalanceConstants.DELAY_TIME, DebounceType.kFalling);

    private MovingAverageFilter filter;

    private SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()};


    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        filter = new MovingAverageFilter(5);

        DataLogger.addDataElement("magnitude", () -> magnitude);
        DataLogger.addDataElement("magnitudeROC", () -> magnitudeRateOfChange);
        DataLogger.addDataElement("filtered magnitudeROC", () -> filteredMagnitudeRateOfChange);
        DataLogger.addDataElement("pitch", () -> pitchAngle);
        DataLogger.addDataElement("roll", () -> rollAngle);

        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {}


    @Override
    public void execute() {

        magnitudeRateOfChange = magnitude - lastMagnitude;

        lastMagnitude = magnitude;

        filteredMagnitudeRateOfChange = filter.filter(magnitudeRateOfChange);

        pitchAngle = drivetrain.getPitch2d().getDegrees();
        rollAngle = drivetrain.getRoll2d().getDegrees();
        theta = LightningMath.inputModulus(-(Math.atan2(rollAngle, pitchAngle) + Math.PI), -Math.PI,
                Math.PI);
        magnitude = Math.sqrt((pitchAngle * pitchAngle) + (rollAngle * rollAngle));

        speedMetersPerSecond = MathUtil.clamp(magnitude * AutoBalanceConstants.MAGNITUDE_SCALER,
                AutoBalanceConstants.MIN_MAGNITUDE_THRESHOLD,
                AutoBalanceConstants.MAX_MAGNITUDE_THRESHOLD);

        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(theta));

        }

        if (delay.calculate(magnitude > AutoBalanceConstants.MAGNITUDE_THRESHOLD
                && filteredMagnitudeRateOfChange < AutoBalanceConstants.MAGNITUDE_RATE_OF_CHANGE_THRESHOLD)) {
            drivetrain.setStates(moduleStates);
        } else {
            drivetrain.stop();
        }

        SmartDashboard.putNumber("magROC", filteredMagnitudeRateOfChange);
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
