package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

public class Wrist extends SubsystemBase {

    // The motor, encoder, and PID controller
    private CANSparkMax motor;
    private PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    private PowerDistribution PDH = new PowerDistribution(21, ModuleType.kRev);

    private SparkMaxAbsoluteEncoder encoder;

    // The encoder offset
    private double OFFSET;

    // The target angle to be set to the wrist
    private double targetAngle;
    private double minPower;

    private boolean isBrakeModeSet = true;

    private Arm arm;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Wrist(Arm arm) {
        this.arm = arm;
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
        // encoder = NeoConfig.createAbsoluteEncoder(motor, OFFSET);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        // encoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);

        targetAngle = getAngle().getDegrees();

        // Initialize the shuffleboard values and start logging data
        motor.getReverseLimitSwitch(WristConstants.BOTTOM_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
        motor.getForwardLimitSwitch(WristConstants.TOP_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);

        initializeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Method to update the shuffleboard
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Wrist", WristConstants.LOG_PERIOD, new Pair<String, Object>("Wrist Target Angle", (DoubleSupplier) () -> targetAngle),
                new Pair<String, Object>("Wrist angle", (DoubleSupplier) () -> getAngle().getDegrees()), new Pair<String, Object>("Wrist on target", (BooleanSupplier) () -> onTarget()),
                new Pair<String, Object>("Wrist motor temperature", (DoubleSupplier) () -> motor.getMotorTemperature()),
                new Pair<String, Object>("Wrist Motor Controller Output (Amps)", (DoubleSupplier) () -> motor.getOutputCurrent()),
                new Pair<String, Object>("Wrist fwd Limit", (BooleanSupplier) () -> getTopLimitSwitch()), new Pair<String, Object>("Wrist rev Limit", (BooleanSupplier) () -> getBottomLimitSwitch()));
    }

    /**
     * Gets the angle of the wrist from the encoder
     * 
     * @return Rotation2d of the wrist from encoder
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(encoder.getPosition() * WristConstants.POSITION_CONVERSION_FACTOR - OFFSET, -180, 180));
    }

    public Rotation2d getGroundRelativeAngle(Rotation2d armAngle) {
        return armAngle.plus(getAngle());
    }

    /**
     * Takes a rotation2d and sets the wrist to that angle bounded by the min and max angles
     * 
     * @param angle Rotation2d to set the wrist to
     */
    public void setAngle(Rotation2d angle) {
        targetAngle = MathUtil.clamp(angle.getDegrees(), WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
        // motor.set(controller.calculate(getAngle().getDegrees(), targetAngle) +  LightningShuffleboard.getDouble("Wrist", "kF", 0) * getGroundRelativeAngle(arm.getAngle()).getDegrees());
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
        // return true;
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
        // return true;
    }

    @Override
    public void periodic() {

        // controller.setP(LightningShuffleboard.getDouble("Lift", "wrist kP", WristConstants.kP));
        // setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Lift", "wrist setpoint", -90)));
        // LightningShuffleboard.setDouble("Lift", "ground relative wrist angle", getGroundRelativeAngle(arm.getAngle()).getDegrees());
        LightningShuffleboard.setDouble("Lift", "wrist relative wrist angle", getAngle().getDegrees());

        if (Math.abs(controller.getPositionError()) > 2) {
            minPower = Math.signum(controller.getPositionError()) * -0.035;
        } else {
            minPower = 0;
        }

        double POutput = controller.calculate(getAngle().getDegrees(), targetAngle);
        double FOutput = WristConstants.WRIST_KF_MAP.get(getGroundRelativeAngle(arm.getAngle()).getDegrees()) * getGroundRelativeAngle(arm.getAngle()).getDegrees();
        double power = POutput + FOutput + minPower;
        motor.set(power);

        // LightningShuffleboard.setDouble("Wrist", "FF output", FOutput);
        // LightningShuffleboard.setDouble("Wrist", "PID output", POutput);
        // LightningShuffleboard.setDouble("Wrist", "minpower", minPower);
        LightningShuffleboard.setDouble("Wrist", "wrist output power", power);

        periodicShuffleboard.loop();
    }
}
