package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.PIDDashboardTuner;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    private double lastAngle = -90;
    private double lastTime = 0;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ArmConstants.PROFILED_MAX_VELOCITY, ArmConstants.PROFILED_MAX_ACCEL);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Arm() {
        if (Constants.isBlackout()) {
            // if blackout, use the blackout offset
            OFFSET = ArmConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            // otherwise, assume gridlock offset
            OFFSET = ArmConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        motor = NeoConfig.createMotor(RobotMap.CAN.ARM_MOTOR, ArmConstants.MOTOR_INVERT,
                ArmConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE, ArmConstants.MOTOR_TYPE,
                ArmConstants.NEUTRAL_MODE);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

        controller = NeoConfig.createPIDController(motor.getPIDController(),
                new SparkMaxPIDGains(ArmConstants.DOWN_kP, ArmConstants.DOWN_kI,
                        ArmConstants.DOWN_kD, ArmConstants.DOWN_kF),
                new SparkMaxPIDGains(ArmConstants.UP_kP, ArmConstants.UP_kI, ArmConstants.UP_kD,
                        ArmConstants.UP_kF),
                encoder);
        controller.setOutputRange(0.05, 1, 1);
        controller.setOutputRange(-1, -0.05, 0);

        // controller = NeoConfig.createPIDController(motor.getPIDController(),
        // new SparkMaxPIDGains(ArmConstants.PROFILED_kP, ArmConstants.PROFILED_kI,
        // ArmConstants.PROFILED_kD, ArmConstants.PROFILED_kF),
        // encoder);
        encoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
        // encoder.setZeroOffset(-194);
        // controller.setOutputRange(ArmConstants.MIN_POWER, ArmConstants.MAX_POWER);
        motor.setClosedLoopRampRate(0);



        // PIDDashboardTuner tuner = new PIDDashboardTuner("Arm", controller);

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Arm target angle", () -> targetAngle);
        DataLogger.addDataElement("Arm angle", () -> getAngle().getDegrees());
        DataLogger.addDataElement("Arm on target", () -> onTarget() ? 1 : 0);
        DataLogger.addDataElement("Arm motor temperature", () -> motor.getMotorTemperature());
        DataLogger.addDataElement("Arm Motor Controller Input Voltage",
                () -> motor.getBusVoltage());
        DataLogger.addDataElement("Arm Motor Controller Output (Amps)",
                () -> motor.getOutputCurrent());

        DataLogger.addDataElement("encoder velocity", () -> encoder.getVelocity());
        DataLogger.addDataElement("encoder angle", () -> getAngle().getDegrees());
        DataLogger.addDataElement("output power", () -> motor.get());

    }

    /**
     * SetAngle: sets the angle of the arm to the angle in the given Rotation2d object
     *
     * @param angle a Rotation2d object containing the angle to set the arm to
     *
     */
    public void setAngle(Rotation2d angle) {
        targetAngle =
                MathUtil.clamp(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);

        var profile = new TrapezoidProfile(constraints, goal);
        setpoint = profile.calculate(0.02);

        // LightningShuffleboard.setDouble("Arm", "target agle", targetAngle);

        if (targetAngle - getAngle().getDegrees() > 2) {
            controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 1);
        } else {
            controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 0);
        }

        // controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 1);

    }

    // public void setAngle(Rotation2d input) {
    // // targetAngle = MathUtil.clamp(input.getDegrees(), ArmConstants.MIN_ANGLE,
    // // ArmConstants.MAX_ANGLE);
    // var profile = new TrapezoidProfile(constraints, goal);
    // setpoint = profile.calculate(0.02);
    // // controller.setReference(setpoint.velocity, ControlType.kVelocity);


    // LightningShuffleboard.setDouble("Arm", "target velocity", setpoint.velocity);
    // }

    /**
     * GetAngle
     * 
     * @return the angle of the arm as a Rotation2d object
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition() - OFFSET);
    }

    /**
     * SetPower: sets the percent power of the arm motor
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
     * getBottomLimitSwitch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimitSwitch(ArmConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * getTopLimitSwitch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimitSwitch(ArmConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * onTarget
     * 
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < ArmConstants.TOLERANCE;
    }

    public boolean isReachable(Rotation2d angle) {
        return angle.getDegrees() >= ArmConstants.MIN_ANGLE
                && angle.getDegrees() <= ArmConstants.MAX_ANGLE;
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Arm", "fwd Limit", getForwardLimitSwitch());
        LightningShuffleboard.setBool("Arm", "rev Limit", getReverseLimitSwitch());
        LightningShuffleboard.setDouble("Arm", "absolute encoder", getAngle().getDegrees());
        // setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Arm", "setpoint",
        // -90)));

        double outputVeloc = LightningShuffleboard.getDouble("Arm", "velocity target", 0);

        if (outputVeloc > 0) {
            controller.setReference(outputVeloc, CANSparkMax.ControlType.kVelocity, 1);
        } else {
            controller.setReference(outputVeloc, CANSparkMax.ControlType.kVelocity, 0);
        }

        LightningShuffleboard.setDouble("Arm", "current velocity", encoder.getVelocity());

        // LightningShuffleboard.setDouble("Arm", "goal veloc", controller.getGoal().velocity);
        // LightningShuffleboard.setDouble("Arm", "goal pos", controller.getGoal().position);


        // LightningShuffleboard.setDouble("Arm", "pos error", controller.getPositionError());
        // LightningShuffleboard.setDouble("Arm", "error", controller.getPositionError());
        // controller.setFF(LightningShuffleboard.getDouble("Arm", "up kF", ArmConstants.UP_kF), 1);
        controller.setP(LightningShuffleboard.getDouble("Arm", "down kP", ArmConstants.DOWN_kP), 0);
        controller.setP(LightningShuffleboard.getDouble("Arm", "up kP", ArmConstants.UP_kP), 1);
        // controller.setFF(LightningShuffleboard.getDouble("Arm", "down kF", ArmConstants.DOWN_kF),
        // 0);


        controller.setFF(Math.signum(getAngle().getCos())
                * Math.pow(Math.abs(getAngle().getCos()),
                        LightningShuffleboard.getDouble("Arm", "exponent", .5))
                * LightningShuffleboard.getDouble("Arm", "up magnitude", 0.022), 1);
        controller.setFF(
                getAngle().getCos() * LightningShuffleboard.getDouble("Arm", "down magnitude", 0.1)
                        + 0.01,
                0);

        // motor.setClosedLoopRampRate(LightningShuffleboard.getDouble("Arm", "ramp rate", 1));

        // LightningShuffleboard.setDouble("Arm", "curr speed", motor.get());
        LightningShuffleboard.setDouble("Arm", "output power", motor.getAppliedOutput());
    }
}
