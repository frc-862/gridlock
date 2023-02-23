package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
        controller = NeoConfig.createPIDController(motor.getPIDController(), new SparkMaxPIDGains(
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kF), encoder);
        encoder.setPositionConversionFactor(360);
        // encoder.setZeroOffset(-194);
        controller.setOutputRange(ArmConstants.MIN_POWER, ArmConstants.MAX_POWER);
        motor.setClosedLoopRampRate(2);


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

        controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 0);

    }

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

    /**
     * onTarget
     * 
     * @param target the target to check against
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget(double target) {
        return Math.abs(getAngle().getDegrees() - target) < ArmConstants.TOLERANCE;
    }

    public boolean isReachable(Rotation2d angle) {
        return angle.getDegrees() >= ArmConstants.MIN_ANGLE
                && angle.getDegrees() <= ArmConstants.MAX_ANGLE;
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Arm", "fwd Limit", getForwardLimitSwitch());
        LightningShuffleboard.setBool("Arm", "rev Limit", getReverseLimitSwitch());

        LightningShuffleboard.setDouble("Lift", "arm angle", getAngle().getDegrees());

        // LightningShuffleboard.setBool("Lift", "Arm on target", onTarget());
        // LightningShuffleboard.setDouble("Lift", "Arm target", targetAngle);


        // setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Lift", "arm setpoint", -90)));

        // double kf = LightningShuffleboard.getDouble("Lift", "kF", ArmConstants.kF);
        // double kp = LightningShuffleboard.getDouble("Lift", "kP", ArmConstants.kP);
        // controller.setP(kp, 0);
        // System.out.println(controller.setFF(kf, 0));

        controller.setFF(ArmConstants.ARM_UP_KF_MAP.get(getAngle().getDegrees()), 0);
    }
}
