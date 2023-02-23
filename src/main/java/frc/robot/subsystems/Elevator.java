package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.PIDDashboardTuner;

public class Elevator extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private RelativeEncoder encoder;
    private double targetExtension;

    public Elevator() {
        motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, ElevatorConstants.MOTOR_INVERT,
                ElevatorConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE,
                ElevatorConstants.MOTOR_TYPE, ElevatorConstants.NEUTRAL_MODE);
        encoder = NeoConfig.createBuiltinEncoder(motor);
        controller =
                NeoConfig.createPIDController(
                        motor.getPIDController(), new SparkMaxPIDGains(ElevatorConstants.kP,
                                ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kF),
                        encoder);
        encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        controller.setOutputRange(ElevatorConstants.MIN_POWER, ElevatorConstants.MAX_POWER);

        encoder.setPosition(0);

        // input inches per second
        double maxVelocity = 0.003;
        // ((1 / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI) / ElevatorConstants.GEAR_RATIO)
        // / 60) * 10;
        controller.setSmartMotionMaxVelocity(maxVelocity, 0);
        controller.setSmartMotionMaxAccel(maxVelocity * 2, 0);
        controller.setSmartMotionAllowedClosedLoopError(0.05, 0);
        controller.setSmartMotionMinOutputVelocity(0, 0);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        motor.setClosedLoopRampRate(.3);

        initLogging();

        // PIDDashboardTuner tuner = new PIDDashboardTuner("Elevator", controller);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Elevator Extension", () -> getExtension());
        DataLogger.addDataElement("Elevator Target Height", () -> targetExtension);
        DataLogger.addDataElement("Elevator on Target", () -> onTarget() ? 1 : 0);
        DataLogger.addDataElement("bottom limit switch", () -> getBottomLimitSwitch() ? 1 : 0);
        DataLogger.addDataElement("top limit switch", () -> getTopLimitSwitch() ? 1 : 0);

        DataLogger.addDataElement("Elevator Motor Temperature", () -> motor.getMotorTemperature());
        DataLogger.addDataElement("Elevator Motor Output Current", () -> motor.getOutputCurrent());
        DataLogger.addDataElement("Elevator Motor Controller Output (Amps)",
                () -> motor.getOutputCurrent());
        DataLogger.addDataElement("Elevator Motor Controller Input Voltage",
                () -> motor.getBusVoltage());

    }

    /**
     * getExtension
     * 
     * @return the extension distance of the elevator in inches
     */
    public double getExtension() {
        return encoder.getPosition();
    }

    /**
     * setExtension
     * 
     * @param target the target distance in inches
     */
    public void setExtension(double target) {
        // if the target is reachable, set the target and enable the controller
        targetExtension = MathUtil.clamp(target, ElevatorConstants.MIN_EXTENSION,
                ElevatorConstants.MAX_EXTENSION);
        controller.setReference(targetExtension, CANSparkMax.ControlType.kPosition, 0);

        // otherwise, do nothing
    }

    /**
     * setPower
     * 
     * @param power the percent speed to set the elevator motor to
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
     * onTarget
     * 
     * @return true if the elevator is within the tolerance of the target
     */
    public boolean onTarget() {
        return Math.abs(targetExtension - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
    }

    /**
     * onTarget
     * @param target the target to check against
     * @return true if the elevator is within the tolerance of the target
     */
    public boolean onTarget(double target) {
        return Math.abs(target - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
    }

    /**
     * getBottomLimitSwitch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean getBottomLimitSwitch() {
        return motor.getReverseLimitSwitch(ElevatorConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * getTopLimitSwitch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getTopLimitSwitch() {
        return motor.getForwardLimitSwitch(ElevatorConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * setEncoderPosition
     *
     * @param position the position to set the encoder to in inches
     */
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    /**
     * isReachable
     * 
     * @param targetHeight the target height in inches
     * 
     * @return true if the target height is reachable by the elevator
     */
    public boolean isReachable(double targetHeight) {
        // return targetHeight >= (ElevatorConstants.MIN_EXTENSION
        // + ElevatorConstants.ELEVATOR_HEIGHT_OFFSET)
        // && targetHeight <= (ElevatorConstants.MAX_EXTENSION
        // + ElevatorConstants.ELEVATOR_HEIGHT_OFFSET);
        return true;
    }

    @Override
    public void periodic() {
        // if (getTopLimitSwitch()) {
        // encoder.setPosition(ElevatorConstants.MAX_EXTENSION);
        // }

        // if (getBottomLimitSwitch()) {
        // encoder.setPosition(ElevatorConstants.MIN_EXTENSION);
        // }

        LightningShuffleboard.setBool("Elevator", "Top Limit", getTopLimitSwitch());
        LightningShuffleboard.setBool("Elevator", "Bottom Limit", getBottomLimitSwitch());
        LightningShuffleboard.setDouble("Elevator", "Elevator Height", getExtension());

        LightningShuffleboard.setBool("Lift", "Elevator on target", onTarget());
        LightningShuffleboard.setDouble("Lift", "Elevator target", targetExtension);


        // setExtension(LightningShuffleboard.getDouble("Elevator", "target elevator height", 0));
        // controller.setP(LightningShuffleboard.getDouble("Elevator", "KP", controller.getP()));
        // controller.setFF(LightningShuffleboard.getDouble("Elevator", "KF", controller.getFF()));

        // controller.setSmartMotionMaxVelocity(
        // LightningShuffleboard.getDouble("Elevator", "max velocity set", 0.02), 0);

        // LightningShuffleboard.setString("Elevator", "strategy",
        // controller.getSmartMotionAccelStrategy(0).toString());
        // LightningShuffleboard.setDouble("Elevator", "mac veloc",
        // controller.getSmartMotionMaxVelocity(0));
        // LightningShuffleboard.setDouble("Elevator", "min veloc",
        // controller.getSmartMotionMinOutputVelocity(0));
        // LightningShuffleboard.setDouble("Elevator", "max accel",
        // controller.getSmartMotionMaxAccel(0));
        // LightningShuffleboard.setDouble("Elevator", "max error",
        // controller.getSmartMotionAllowedClosedLoopError(0));

        // motor.setClosedLoopRampRate(LightningShuffleboard.getDouble("Elevator", "ramp rate",
        // .3));

        LightningShuffleboard.setDouble("Elevator", "curr speed", motor.get());

    }
}
