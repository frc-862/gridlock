package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Wrist extends SubsystemBase {

    // The motor, encoder, and PID controller
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;

    // The encoder offset
    private double OFFSET;

    // The target angle to be set to the wrist
    private double targetAngle;

    public Wrist() {
        // If blackout, use the blackout offset
        if (Constants.isBlackout()) {
            OFFSET = WristConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            // Otherwise, assume gridlock offset
            OFFSET = WristConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        // Create the motor and configure it
        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT, WristConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMPENSATION, WristConstants.MOTOR_TYPE,
                WristConstants.NEUTRAL_MODE);
        motor.setClosedLoopRampRate(2);

        // Create the absolute encoder and sets the conversion factor
        encoder = NeoConfig.createAbsoluteEncoder(motor, OFFSET);
        encoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);

        // Create the PID controller and set the output range
        controller = NeoConfig.createPIDController(motor.getPIDController(), new SparkMaxPIDGains(WristConstants.DOWN_kP, WristConstants.DOWN_kI, WristConstants.DOWN_kD, WristConstants.DOWN_kF),
                new SparkMaxPIDGains(WristConstants.UP_kP, WristConstants.UP_kI, WristConstants.UP_kD, WristConstants.UP_kF), encoder);
        controller.setOutputRange(WristConstants.MIN_POWER, WristConstants.MAX_POWER);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Method to update the shuffleboard
    private void updateShuffleboard() {
        LightningShuffleboard.setDouble("Wrist", "Wrist Target angle", targetAngle);
        LightningShuffleboard.setDouble("Wrist", "Wrist angle", getAngle().getDegrees());
        LightningShuffleboard.setBool("Wrist", "Wrist on target", onTarget());
        LightningShuffleboard.setDouble("Wrist", "Wrist motor temperature", motor.getMotorTemperature());
        LightningShuffleboard.setDouble("Wrist", "Wrist Motor Controller Output (Amps)", motor.getOutputCurrent());
        LightningShuffleboard.setDouble("Wrist", "Wrist Motor Controller Input Voltage", motor.getBusVoltage());
        LightningShuffleboard.setBool("Wrist", "Wrist fwd Limit", getTopLimitSwitch());
        LightningShuffleboard.setBool("Wrist", "Wrist rev Limit", getBottomLimitSwitch());
    }

    /**
     * Gets the angle of the wrist from the encoder
     * 
     * @return Rotation2d of the wrist from encoder
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition() - OFFSET);
    }

    /**
     * Takes a rotation2d and sets the wrist to that angle bounded by the min and max angles
     * 
     * @param angle Rotation2d to set the wrist to
     */
    public void setAngle(Rotation2d angle) {
        targetAngle = MathUtil.clamp(angle.getDegrees(), WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
    }

    /**
     * Sets the raw power to the wrist from -1 to 1
     */
    public void setPower(double power) {
        motor.set(MathUtil.clamp(power, WristConstants.MIN_POWER, WristConstants.MAX_POWER));
    }

    /**
     * stops the writs motor
     */
    public void stop() {
        motor.set(0d);
    }

    /**
     * Gets the bottom limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getBottomLimitSwitch() {
        return motor.getReverseLimitSwitch(WristConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Gets the top limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getTopLimitSwitch() {
        return motor.getForwardLimitSwitch(WristConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Checks if the wrist is on target with the set angle
     * 
     * @return the encoder position
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < WristConstants.TOLERANCE;
    }

    /**
     * Checks if the wrist is within the tolerance of the target angle
     * 
     * @param target the target to check against
     * 
     * @return true if the wrist is within the tolerance of the target angle
     */
    public boolean onTarget(double target) {
        return Math.abs(getAngle().getDegrees() - target) < WristConstants.TOLERANCE;
    }

    @Override
    public void periodic() {

        // If were not on target
        if (!onTarget()) {
            // Checks if wrist is going up or down
            if (targetAngle - getAngle().getDegrees() > 2) {
                // Uses the up gains
                controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 1);
            } else {
                // Uses the down gains
                controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 0);
            }
        }

        // Starts logging and updates the shuffleboard
        updateShuffleboard();

    }
}
