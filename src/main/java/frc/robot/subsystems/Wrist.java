package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Wrist extends SubsystemBase {

    // The motor, encoder, and PID controller
    private CANSparkMax motor;
    private PIDController upController = new PIDController(WristConstants.UP_kP, WristConstants.kI, WristConstants.UP_kD);
    private PIDController downController = new PIDController(WristConstants.DOWN_kP, WristConstants.kI, WristConstants.DOWN_kD);

    private SparkMaxAbsoluteEncoder encoder;

    // The encoder offset
    private double OFFSET;

    // The target angle to be set to the wrist
    private double targetAngle ;
    private double currentAngle;
    private double PIDOutput;
    private double FOutput;

    double tolerance = WristConstants.TOLERANCE;

    private Arm arm;

    private boolean disableWrist = false;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Wrist(Arm arm) {
        this.arm = arm;

        // If blackout, use the blackout offset
        if (Constants.isBlackout()) {
            OFFSET = WristConstants.ENCODER_OFFSET_BLACKOUT;
        } else { // Otherwise, assume gridlock offset
            OFFSET = WristConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        // Create the motor and configure it
        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT, WristConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMPENSATION, WristConstants.MOTOR_TYPE,
                WristConstants.NEUTRAL_MODE);
        motor.setClosedLoopRampRate(2);

        // Create the absolute encoder and sets the conversion factor
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

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
        periodicShuffleboard = new LightningShuffleboardPeriodic("Wrist", WristConstants.LOG_PERIOD, 
            new Pair<String, Object>("Wrist Target Angle", (DoubleSupplier) () -> targetAngle),
            new Pair<String, Object>("Wrist angle", (DoubleSupplier) () -> getAngle().getDegrees()),
            new Pair<String, Object>("Wrist motor temperature", (DoubleSupplier) () -> motor.getMotorTemperature()),
            new Pair<String, Object>("Wrist on target", (BooleanSupplier) () -> onTarget()),
            new Pair<String, Object>("Wrist Motor Controller Output (Amps)", (DoubleSupplier) () -> motor.getOutputCurrent()),
            new Pair<String, Object>("built in position", (DoubleSupplier) () -> motor.getEncoder().getPosition()),
            new Pair<String, Object>("faults", (DoubleSupplier) () -> (double) motor.getFaults()));
            // new Pair<String, Object>("Wrist fwd Limit", (BooleanSupplier) () -> getTopLimitSwitch()), 
            // new Pair<String, Object>("Wrist rev Limit", (BooleanSupplier) () -> getBottomLimitSwitch()));
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
    }

    /**
     * Sets the raw power to the wrist from -1 to 1
     */
    public void setPower(double power) {
        motor.set(MathUtil.clamp(power, WristConstants.MIN_POWER, WristConstants.MAX_POWER));
    }

    /**
     * stops the wrist's motor
     */
    public void stop() {
        motor.set(0d);
    }

    /**
     * Gets the bottom limit switch
     * 
     * @return true if the bottom limit switch is pressed
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
     * @return The target angle of the wrist
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Checks if the wrist is on target with the set angle
     * 
     * @return the encoder position
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < tolerance;
    }

    /**
     * Checks if the wrist is within the tolerance of the target angle
     * 
     * @param target the target to check against
     * 
     * @return true if the wrist is within the tolerance of the target angle
     */
    public boolean onTarget(double target) {
        return Math.abs(getAngle().getDegrees() - target) < tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void disableWrist() {
        disableWrist = true;
    }

    @Override
    public void periodic() {

        // FOR tuning and testing
        // upController.setP(LightningShuffleboard.getDouble("Lift", "up wrist kP", WristConstants.UP_kP));
        // downController.setP(LightningShuffleboard.getDouble("Lift", "down wrist kP", WristConstants.DOWN_kP));
        // upController.setD(LightningShuffleboard.getDouble("Lift", "wrist up D", WristConstants.UP_kD));
        // downController.setD(LightningShuffleboard.getDouble("Lift", "wrist down D", WristConstants.DOWN_kD));

        // setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Lift", "wrist setpoint", getAngle().getDegrees())));

        LightningShuffleboard.setDouble("Lift", "GR wrist angle", getGroundRelativeAngle(arm.getAngle()).getDegrees());

        currentAngle = getAngle().getDegrees();

        if (arm.getAngle().getDegrees() < 90) { // This check that the arm is in front of the robot
            if (targetAngle - currentAngle > 0) {
                PIDOutput = upController.calculate(currentAngle, targetAngle);
            } else {
                PIDOutput = downController.calculate(currentAngle, targetAngle);
            }
        } else { // This is for behind the robot
            if (targetAngle - currentAngle > 0) {
                PIDOutput = downController.calculate(currentAngle, targetAngle);
            } else {
                PIDOutput = upController.calculate(currentAngle, targetAngle);
            }
        }

        FOutput = WristConstants.WRIST_KF_MAP.get(arm.getAngle().getDegrees());

        if (disableWrist) {
            setPower(0);
        } else {
            motor.set(PIDOutput + FOutput);
        }

        periodicShuffleboard.loop();
    }
}
