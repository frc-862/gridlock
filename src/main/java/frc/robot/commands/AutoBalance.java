package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.filter.MovingAverageFilter;
import frc.thunder.logging.DataLogger;
import frc.thunder.math.LightningMath;
import frc.thunder.shuffleboard.LightningShuffleboard;

/*
 * This command is used to balance the robot on the climb. It uses the IMU to determine the pitch
 * and roll of the robot. It then uses a PID controller to determine the speed of the robot to keep
 * it balanced.
 */
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
    private double timer;

    private MovingAverageFilter filter;

    private SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private PIDController controller = new PIDController(AutoBalanceConstants.kP, AutoBalanceConstants.kI, AutoBalanceConstants.kD);

    private enum climbStates {
        CLIMB, CHECK_FALLING, FALLING, STOP
    }

    private climbStates climbState;

    /**
     * This command is used to balance the robot on the climb. It uses the IMU to determine the pitch
     * and roll of the robot. It then uses a PID controller to determine the speed of the robot to keep
     * it balanced.
     * 
     * @param drivetrain The drivetrain
     */
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
    public void initialize() {
        climbState = climbStates.CLIMB;
        drivetrain.resetOdometry(new Pose2d(new Translation2d(2.75, drivetrain.getPose().getY()), drivetrain.getPose().getRotation()));
    }

    @Override
    public void execute() {

        pitchAngle = drivetrain.getPitch2d().getDegrees();
        rollAngle = drivetrain.getRoll2d().getDegrees();
        theta = LightningMath.inputModulus(-(Math.atan2(rollAngle, pitchAngle) + Math.PI), -Math.PI, Math.PI);
        magnitude = Math.sqrt((pitchAngle * pitchAngle) + (rollAngle * rollAngle));

        controller.setP(LightningShuffleboard.getDouble("autoBalance", "pee", AutoBalanceConstants.kP));

        if (magnitude < AutoBalanceConstants.BALANCED_MAGNITUDE) {
            speedMetersPerSecond = 0;
        } else {
            speedMetersPerSecond = MathUtil.clamp(controller.calculate(drivetrain.getPose().getX(), AutoBalanceConstants.TARGET_X), AutoBalanceConstants.MIN_SPEED_THRESHOLD,
                    AutoBalanceConstants.MAX_SPEED_THRESHOLD);
        }

        LightningShuffleboard.setDouble("autoBalance", "speed", speedMetersPerSecond);
        LightningShuffleboard.setDouble("autoBalance", "error", controller.getPositionError());

        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(theta));

        }

        switch (climbState) {
            case CLIMB:
                if (magnitude > AutoBalanceConstants.LOWER_MAGNITUDE_THRESHOLD) {
                    drivetrain.setStates(moduleStates);
                }
                if (magnitude > AutoBalanceConstants.UPPER_MAGNITUDE_THRESHOLD) {
                    climbState = climbStates.CHECK_FALLING;
                }

                break;
            case CHECK_FALLING:
                drivetrain.setStates(moduleStates);

                if (magnitude < AutoBalanceConstants.UPPER_MAGNITUDE_THRESHOLD) {
                    climbState = climbStates.FALLING;
                    timer = Timer.getFPGATimestamp();
                }

                if (magnitude < AutoBalanceConstants.BALANCED_MAGNITUDE) {
                    climbState = climbStates.STOP;
                }

                break;
            case FALLING:
                drivetrain.stop();

                if (Timer.getFPGATimestamp() - timer > AutoBalanceConstants.DELAY_TIME) {
                    climbState = climbStates.CLIMB;
                }

                break;
            case STOP:
                drivetrain.stop();
                break;
        }
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
