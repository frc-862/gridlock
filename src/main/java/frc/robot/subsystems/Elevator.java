package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The elevator subsystem
 */
public class Elevator extends SubsystemBase {

    // The motor, encoder, and PID controller
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private RelativeEncoder encoder;

    // The target extension to be set to the elevator
    private double targetExtension;

    // Periodic Shuffleboard 
    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Elevator() {

        // Create the motor and configure it
        motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, ElevatorConstants.MOTOR_INVERT, ElevatorConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMPENSATION, ElevatorConstants.MOTOR_TYPE,
                ElevatorConstants.NEUTRAL_MODE);

        // Create the relative encoder and sets the conversion factor
        encoder = NeoConfig.createBuiltinEncoder(motor);
        encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        encoder.setPosition(0);

        // Create the PID controller and set the output range
        controller = NeoConfig.createPIDController(motor.getPIDController(), new SparkMaxPIDGains(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kF), encoder);
        controller.setOutputRange(ElevatorConstants.MIN_POWER, ElevatorConstants.MAX_POWER);

        targetExtension = getExtension();

        initializeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Metod to starts logging and updates the shuffleboard
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Elevator", ElevatorConstants.LOG_PERIOD, new Pair<String, Object>("Top Limit", (BooleanSupplier) () -> getTopLimitSwitch()),
                new Pair<String, Object>("Bottom Limit", (BooleanSupplier) () -> getBottomLimitSwitch()), new Pair<String, Object>("Elevator target height", (DoubleSupplier) () -> targetExtension),
                new Pair<String, Object>("Elevator height", (DoubleSupplier) () -> getExtension()), new Pair<String, Object>("Elevator on target", (BooleanSupplier) () -> onTarget()),
                new Pair<String, Object>("Elevator motor temperature", (DoubleSupplier) () -> motor.getMotorTemperature()),
                new Pair<String, Object>("Elevator motor controller output (volts)", (DoubleSupplier) () -> motor.getAppliedOutput()),
                new Pair<String, Object>("Elevator motor controller output (Amps)", (DoubleSupplier) () -> motor.getOutputCurrent()),
                new Pair<String, Object>("Elevator motor controller voltage", (DoubleSupplier) () -> motor.getBusVoltage()));

    }

    /**
     * Method to get the current extension of the elevator
     * 
     * @return the extension distance of the elevator in inches
     */
    public double getExtension() {
        return encoder.getPosition();
    }

    /**
     * Method to set Extension of the elevator to a target
     * 
     * @param target the target distance in inches
     */
    public void setExtension(double target) {
        // if the target is reachable, set the target and enable the controller
        targetExtension = MathUtil.clamp(target, ElevatorConstants.MIN_EXTENSION, ElevatorConstants.MAX_EXTENSION);
        controller.setReference(targetExtension, CANSparkMax.ControlType.kPosition, 0);

        // otherwise, do nothing
    }

    /**
     * Method to set the power to the elvevator
     * 
     * @param power the percent power from -1 to 1
     */
    public void setPower(double power) {
        motor.set(MathUtil.clamp(power, ElevatorConstants.MIN_POWER, ElevatorConstants.MAX_POWER));
    }

    /**
     * stop set the elevator motor to 0% output
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * Checks if the elevator is within the tolerance of the target extension
     * 
     * @return true if the elevator is within the tolerance of the target extension
     */
    public boolean onTarget() {
        return Math.abs(targetExtension - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
        // return true;
    }

    /**
     * Checks if the elevator is within the tolerance of the target extension
     * 
     * @param target the target to check against
     * 
     * @return true if the elevator is within the tolerance of the target extension
     */
    public boolean onTarget(double target) {
        return Math.abs(target - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
        // return true;
    }

    /**
     * Gets the bottom limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getBottomLimitSwitch() {
        return motor.getReverseLimitSwitch(ElevatorConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Gets the top limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getTopLimitSwitch() {
        return motor.getForwardLimitSwitch(ElevatorConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Sets the encoder position
     *
     * @param position the position to set the encoder to in inches
     */
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    /**
     * Checks if the target height is reachable by the elevator based on min and max heights of the
     * elevator
     * 
     * @param targetHeight the target height in inches
     * 
     * @return true if the target height is reachable by the elevator
     */
    public boolean isReachable(double targetHeight) {
        return targetHeight >= ElevatorConstants.MIN_EXTENSION && targetHeight <= ElevatorConstants.MAX_EXTENSION;
    }

    @Override
    public void periodic() {
        // if (getTopLimitSwitch()) {
        //     encoder.setPosition(ElevatorConstants.MAX_EXTENSION);
        // }

        // if (getBottomLimitSwitch()) {
        //     encoder.setPosition(ElevatorConstants.MIN_EXTENSION);
        // }


        // setExtension(LightningShuffleboard.getDouble("Lift", "ele targ", 0));
    }
}
