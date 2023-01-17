package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.ThetaGains;
import frc.thunder.swervelib.SdsModuleConfigurations;

public class NeoSwerveModule {
    private static final double kModuleMaxAngularVelocity = DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    // Radians per second squared
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

    private double drivePositionConversionFactor = Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * SdsModuleConfigurations.MK4_L2.getDriveReduction();

    // TOOD look at this number when testing
    private double steerPositionConversionFactor = 2.0 * Math.PI * SdsModuleConfigurations.MK4_L2.getSteerReduction();

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final CANCoder m_canCoder;

    private final PIDController m_drivePIDController = new PIDController(Gains.kP, Gains.kI, Gains.kD);

    // TODO see if we want to use a normal pid controller
    private final PIDController m_turningPIDController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

    public NeoSwerveModule(int driveMotorID, int azimuthMotorID, int canCoderID,
            double steerOffset) {

        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.enableVoltageCompensation(DrivetrainConstants.NOMINAL_VOLTAGE);
        m_driveMotor.setSmartCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_turningMotor = new CANSparkMax(azimuthMotorID, MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.enableVoltageCompensation(DrivetrainConstants.NOMINAL_VOLTAGE);
        m_turningMotor.setSmartCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        m_turningMotor.setIdleMode(IdleMode.kBrake);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_canCoder = new CANCoder(canCoderID);
        m_canCoder.configFactoryDefault();
        m_canCoder.configMagnetOffset(steerOffset);
        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        m_driveEncoder.setPositionConversionFactor(drivePositionConversionFactor);
        m_turningEncoder.setPositionConversionFactor(steerPositionConversionFactor);
        m_driveEncoder.setVelocityConversionFactor(drivePositionConversionFactor / 60);
        m_turningEncoder.setVelocityConversionFactor(steerPositionConversionFactor / 60);

        m_turningPIDController.enableContinuousInput(0, 360);
    }

    /**
     * Returns the current state of the module.
     *
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                new Rotation2d(m_canCoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(),
                new Rotation2d(m_canCoder.getPosition()));
    }

    /**
     * Retruns the current angle of the module in degrees
     * 
     * @return the current angle of the module
     */
    public double getSteerAngle() {
        return getState().angle.getDegrees();
    }

    /**
     * Returns the current drive velociry of the module
     * 
     * @return
     */
    public double getDriveVelocity() {
        return getState().speedMetersPerSecond;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_canCoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(Math.toRadians(m_canCoder.getPosition()),
                state.angle.getDegrees());

        // final double turnFeedforward =
        // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().);

        // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput);
    }
}
