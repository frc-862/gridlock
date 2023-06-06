package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The arm subsystem
 */
public class Arm extends SubsystemBase {

    // The motor, encoder, and PID controller
    private CANSparkMax motor;
    private PIDController upController = new PIDController(ArmConstants.UP_kP, ArmConstants.UP_kI, ArmConstants.UP_kD);
    private PIDController downController = new PIDController(ArmConstants.DOWN_kP, ArmConstants.DOWN_kI, ArmConstants.DOWN_kD);
    private SparkMaxAbsoluteEncoder encoder;

    private int currCurrentLimit = ArmConstants.CURRENT_LIMIT;

    // The encoder offset 
    private double OFFSET;

    private double PIDOUT;
    private double kFOut_POSITIONAL;
    private double kFOut_VELOCITY;
    private double power;

    private double tolerance = ArmConstants.TOLERANCE;

    private boolean isDrawingMax = false;
    private double drawMaxTime = 0;

    // The target angle to be set to the arm
    private double targetAngle;

    private boolean disableArm = false;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Arm() {
        if (Constants.isBlackout()) {
            // If blackout, use the blackout offset
            OFFSET = ArmConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            // Otherwise, assume gridlock offset
            OFFSET = ArmConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        // Create the motor and configure it
        motor = NeoConfig.createMotor(RobotMap.CAN.ARM_MOTOR, ArmConstants.MOTOR_INVERT, ArmConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMPENSATION, ArmConstants.MOTOR_TYPE,
                ArmConstants.NEUTRAL_MODE);
        // Sets the ramp rate for the motor
        motor.setClosedLoopRampRate(1);

        // Create the absolute encoder and sets the conversion factor
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);

        motor.getReverseLimitSwitch(ArmConstants.BOTTOM_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
        motor.getForwardLimitSwitch(ArmConstants.TOP_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);

        upController.disableContinuousInput();
        downController.disableContinuousInput();

        // Create the PID controller and set the output range
        targetAngle = getAngle().getDegrees();

        // Starts logging and updates the shuffleboard
        initializeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Metod to starts logging and updates the shuffleboard
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Arm", ArmConstants.LOG_PERIOD, new Pair<String, Object>("Arm angle", (DoubleSupplier) () -> getAngle().getDegrees()),
                new Pair<String, Object>("Arm Target Angle", (DoubleSupplier) () -> targetAngle), new Pair<String, Object>("Arm on target", (BooleanSupplier) () -> onTarget()),
                new Pair<String, Object>("Arm amps", (DoubleSupplier) () -> motor.getOutputCurrent()), new Pair<String, Object>("Arm velocity", (DoubleSupplier) () -> getVelocity()),
                new Pair<String, Object>("built in position", (DoubleSupplier) () -> motor.getEncoder().getPosition()),
                new Pair<String, Object>("faults", (DoubleSupplier) () -> (double) motor.getFaults()));
        // new Pair<String, Object>("Arm Bottom Limit", (BooleanSupplier) () -> getBottomLimitSwitch()),
        // new Pair<String, Object>("Arm Top Limit", (BooleanSupplier) () -> getTopLimitSwitch()), 
        // new Pair<String, Object>("Arm motor controller input voltage", (DoubleSupplier) () -> motor.getBusVoltage()),
        // new Pair<String, Object>("Arm motor controller output (volts)", (DoubleSupplier) () -> motor.getAppliedOutput()));
    }

    /**
     * Sets the angle of the arm to the angle in the given Rotation2d object
     * 
     * @param angle a Rotation2d object containing the angle to set the arm to
     * 
     */
    public void setAngle(Rotation2d angle) {
        targetAngle = MathUtil.clamp(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setCurrentLimit(int currentLimit) {
        if(currentLimit != currCurrentLimit) {
            motor.setSmartCurrentLimit(currentLimit);
        }
        currCurrentLimit = currentLimit;        
    }

    /**
     * Gets the angle gets the angle of the arm
     * 
     * @return the angle of the arm as a Rotation2d object
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(encoder.getPosition() - OFFSET, -140, 220));
        // return Rotation2d.fromDegrees(encoder.getPosition() - OFFSET);
    }

    /**
     * Sets the percent power of the arm motor
     *
     * @param power the percent power to set the arm motor to
     */
    public void setPower(double power) {
        motor.set(MathUtil.clamp(power, ArmConstants.MIN_POWER, ArmConstants.MAX_POWER));
    }

    /**
     * Stop: sets the arm motor to 0% power
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * Gets the bottom limit switch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean getBottomLimitSwitch() {
        return motor.getReverseLimitSwitch(ArmConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Gets the top limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getTopLimitSwitch() {
        return motor.getForwardLimitSwitch(ArmConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * Checks if the arm is within the tolerance of the target angle
     * 
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < tolerance;
    }

    /**
     * Checks if the arm is within the tolerance of the target angle
     * 
     * @param target the target to check against
     * 
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget(double target) {
        return Math.abs(getAngle().getDegrees() - target) < tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Checks if the given angle is reachable by the arm
     * 
     * @param angle the angle to check
     */
    public boolean isReachable(Rotation2d angle) {
        return angle.getDegrees() >= ArmConstants.MIN_ANGLE && angle.getDegrees() <= ArmConstants.MAX_ANGLE;
    }

    public void disableArm() {
        disableArm = true;
    }

    @Override
    public void periodic() {
        double currentAngle = getAngle().getDegrees();
        double currentVelocity = getVelocity();

        // double kFOut = LightningShuffleboard.getDouble("Arm", "kF in", 0);
        kFOut_POSITIONAL = ArmConstants.ARM_POSITIONAL_KF_MAP.get(currentAngle);
        kFOut_VELOCITY = 0d;
        // kFOut_VELOCITY = ArmConstants.ARM_VELOCITY_KF_MAP.get(currentVelocity);

        //swap up and down controllers when in applicable quadrants
        if (currentAngle < 90 && currentAngle > -90) {
            if (targetAngle - currentAngle > 0) {
                PIDOUT = upController.calculate(currentAngle, targetAngle);
            } else {
                PIDOUT = downController.calculate(currentAngle, targetAngle);
            }
        } else {
            if (targetAngle - currentAngle > 0) {
                PIDOUT = downController.calculate(currentAngle, targetAngle);
            } else {
                PIDOUT = upController.calculate(currentAngle, targetAngle);
            }
        }
        power = kFOut_POSITIONAL + kFOut_VELOCITY + PIDOUT;

        if (disableArm) {
            motor.set(0);
        } else {
            motor.set(power);
        }

        if(motor.getOutputCurrent() > 49.5 && !isDrawingMax) {
            isDrawingMax = true;
            drawMaxTime = Timer.getFPGATimestamp();
        } else if(motor.getOutputCurrent() < 49.5){
            isDrawingMax = false;
        }

        if(isDrawingMax) {
            if(Timer.getFPGATimestamp() - drawMaxTime > 2) {
                disableArm = true;
            }
        }

        periodicShuffleboard.loop();

        // upController.setD(LightningShuffleboard.getDouble("Arm", "up kD", ArmConstants.UP_kD));
        // upController.setP(LightningShuffleboard.getDouble("Arm", "up kP", ArmConstants.UP_kP));

        // downController.setD(LightningShuffleboard.getDouble("Arm", "down kD", ArmConstants.DOWN_kD));
        // downController.setP(LightningShuffleboard.getDouble("Arm", "down kP", ArmConstants.DOWN_kP));

        // setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Arm", "arm setpoint", getAngle().getDegrees())));
        LightningShuffleboard.setDouble("Arm", "OUTPUT APPLIED", power);
        // LightningShuffleboard.setDouble("Arm", "kf map", ArmConstants.ARM_KF_MAP.get(currentAngle));
    }

    //TODO make work
    public boolean getSquishToggle() {
        return false;
    }
}
