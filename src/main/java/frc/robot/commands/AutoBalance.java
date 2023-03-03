package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.math.LightningMath;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
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
    private double timer;

    // List of swerve module states to be set to the drivetrain after determining the speed and angle of each motor
    private SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    // PID controller to determine the speed of the robot
    private PIDController controller = new PIDController(AutoBalanceConstants.kP, AutoBalanceConstants.kI, AutoBalanceConstants.kD);

    // States for the climb
    private enum climbStates {
        CLIMB, CHECK_FALLING, FALLING, STOP
    }

    // Current state of the climb
    private climbStates climbState;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    /**
     * This command is used to balance the robot on the climb. It uses the IMU to determine the pitch
     * and roll of the robot. It then uses a PID controller to determine the speed of the robot to keep
     * it balanced.
     * 
     * @param drivetrain The drivetrain
     */
    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        initializeShuffleboard();

        // LightningShuffleboard.setDoubleSupplier("AutoBalance", "magnitude", () -> magnitude);
        // LightningShuffleboard.setDoubleSupplier("AutoBalance","magnitudeROC", () -> magnitudeRateOfChange);
        // LightningShuffleboard.setDoubleSupplier("AutoBalance","filtered magnitudeROC", () -> filteredMagnitudeRateOfChange);
        // LightningShuffleboard.setDoubleSupplier("AutoBalance","pitch", () -> pitchAngle);
        // LightningShuffleboard.setDoubleSupplier("AutoBalance","roll", () -> rollAngle);

        addRequirements(drivetrain);
    }

    // Initializes the shuffleboard
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("AutoBalance", 0.2, new Pair<String, Object>("magnitude", (DoubleSupplier) () -> magnitude),
                // new Pair<String, Object>("magnitudeROC", (DoubleSupplier) () -> magnitudeRateOfChange),
                // new Pair<String, Object>("filtered magnitudeROC", (DoubleSupplier) () -> filteredMagnitudeRateOfChange), 
                new Pair<String, Object>("pitch", (DoubleSupplier) () -> pitchAngle), new Pair<String, Object>("roll", (DoubleSupplier) () -> rollAngle));
    }

    @Override
    public void initialize() {
        // Initialize our climb state to climb
        climbState = climbStates.CLIMB;
        // TODO: get rid of this line after testing and proper calibration of odometry
        // drivetrain.resetOdometry(new Pose2d(new Translation2d(2.75, drivetrain.getPose().getY()), drivetrain.getPose().getRotation()));
    }

    @Override
    public void execute() {

        // Get the pitch and roll of the robot
        pitchAngle = drivetrain.getPitch2d().getDegrees();
        rollAngle = drivetrain.getRoll2d().getDegrees();

        // Calculate the angle of travel based on the pitch and roll
        theta = LightningMath.inputModulus(-(Math.atan2(rollAngle, pitchAngle) + Math.PI), -Math.PI, Math.PI);

        // Calculate the magnitude of the pitch and roll
        magnitude = Math.sqrt((pitchAngle * pitchAngle) + (rollAngle * rollAngle));

        // Tune the auto balance PID controller
        controller.setP(LightningShuffleboard.getDouble("autoBalance", "pee", AutoBalanceConstants.kP));

        // Check if we start falling and if so, stop the robot 
        if (magnitude < AutoBalanceConstants.BALANCED_MAGNITUDE) {
            speedMetersPerSecond = 0;
        } else {
            speedMetersPerSecond = MathUtil.clamp(controller.calculate(drivetrain.getPose().getX(), AutoBalanceConstants.TARGET_X), AutoBalanceConstants.MIN_SPEED_THRESHOLD,
                    AutoBalanceConstants.MAX_SPEED_THRESHOLD);
        }

        // LightningShuffleboard.setDouble("autoBalance", "speed", speedMetersPerSecond);
        // LightningShuffleboard.setDouble("autoBalance", "error", controller.getPositionError());

        // Set the states of the swerve modules
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(theta));

        }

        // Case statement to determine the state of the climb
        switch (climbState) {
            case CLIMB:
                // If were barely on the charge station, set the states of the swerve modules
                if (magnitude > AutoBalanceConstants.LOWER_MAGNITUDE_THRESHOLD) {
                    drivetrain.setStates(moduleStates);
                }

                // Check if were fully on the charge station
                if (magnitude > AutoBalanceConstants.UPPER_MAGNITUDE_THRESHOLD) {
                    // If so, check if we start falling
                    climbState = climbStates.CHECK_FALLING;
                }

                break;
            case CHECK_FALLING:
                drivetrain.setStates(moduleStates);

                // If we start falling, stop the robot and start the timer
                if (magnitude < AutoBalanceConstants.UPPER_MAGNITUDE_THRESHOLD) {
                    climbState = climbStates.FALLING;
                    timer = Timer.getFPGATimestamp();
                }

                // If we are balanced, stop the robot
                if (magnitude < AutoBalanceConstants.BALANCED_MAGNITUDE) {
                    climbState = climbStates.STOP;
                }

                break;
            case FALLING:
                drivetrain.stop();

                // Wait for a certain amount of time before starting to climb again
                if (Timer.getFPGATimestamp() - timer > AutoBalanceConstants.DELAY_TIME) {
                    climbState = climbStates.CLIMB;
                }

                break;
            case STOP:
                drivetrain.stop();
                break;
        }

        periodicShuffleboard.loop();
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
