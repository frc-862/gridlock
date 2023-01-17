package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    private static final double kModuleMaxAngularVelocity =
            DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second
                                                                             // squared

    private double drivePositionConversionFactor =
            Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
                    * SdsModuleConfigurations.MK4_L2.getDriveReduction();

    private double steerPositionConversionFactor =
            2.0 * Math.PI * SdsModuleConfigurations.MK4_L2.getSteerReduction(); // TODO look at this
                                                                                // number when
                                                                                // testing

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final CANCoder m_canCoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController =
            new PIDController(Gains.kP, Gains.kI, Gains.kD);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD, // TODO see if we
                                                                                   // just want a
                                                                                   // PIDContoller
                                                                                   // and not a
                                                                                   // profiled one
                    new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
                            kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward =
            new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);
    private final SimpleMotorFeedforward m_turnFeedforward =
            new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

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
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
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


    public double getSteerAngle() {
        return getState().angle.getDegrees();
    }

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
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(m_canCoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_canCoder.getPosition(),
                state.angle.getRadians());

        final double turnFeedforward =
                m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
