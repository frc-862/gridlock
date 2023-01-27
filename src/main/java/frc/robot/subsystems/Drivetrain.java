package frc.robot.subsystems;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.thunder.swervelib.Mk4ModuleConfiguration;
import frc.thunder.swervelib.Mk4iSwerveModuleHelper;
import frc.thunder.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.thunder.logging.DataLogger;

/**
 * Our drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {

        // Creates our swerve kinematics using the robots track width and wheel base
        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // Creating new pigeon2 gyro
        private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.CAN.PIGEON_ID);


        // Creating our list of module states and module positions
        private SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(),
                        new SwerveModuleState(), new SwerveModuleState()};
        private SwerveModulePosition[] modulePositions =
                        {new SwerveModulePosition(), new SwerveModulePosition(),
                                        new SwerveModulePosition(), new SwerveModulePosition()};

        // Creating new pose, odometry, cahssis speeds
        private Pose2d pose = new Pose2d();
        private SwerveDriveOdometry odometry =
                        new SwerveDriveOdometry(kinematics, getHeading2d(), modulePositions, pose);
        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        // Creating our feed forward
        private final SimpleMotorFeedforward feedForward =
                        new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    
    public double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(342.246);
    public double BACK_LEFT_STEER_OFFSET = -Math.toRadians(122.959);
    public double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(311.660);
    public double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(12.744);


        Path gridlockFile = Paths.get("home/lvuser/gridlock");
        Path blackoutFile = Paths.get("home/lvuser/blackout");

// Creates our drivetrain shuffleboard tab for displaying module data
private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

private final Mk4ModuleConfiguration swerveConfiguration = new Mk4ModuleConfiguration();

public Drivetrain() {
    
        if (Files.exists(blackoutFile)) {
            FRONT_LEFT_STEER_OFFSET = -Math.toRadians(84.832);
            FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(192.7441);
            BACK_LEFT_STEER_OFFSET = -Math.toRadians(19.5996);
            BACK_RIGHT_STEER_OFFSET = -Math.toRadians(63.457);
        }

                // Set our neo module configurations using drive current, steer current, and voltage
                swerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
                swerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
                swerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);

        // Making front left module
        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0),
                swerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR, RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR,
                RobotMap.CAN.FRONT_LEFT_CANCODER, FRONT_LEFT_STEER_OFFSET);

        // Making front right module
        frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                swerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR, RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR,
                RobotMap.CAN.FRONT_RIGHT_CANCODER, FRONT_RIGHT_STEER_OFFSET);

        // Making backleft module
        backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(
                        4, 0),
                swerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR, RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_LEFT_CANCODER, BACK_LEFT_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                swerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR, RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_RIGHT_CANCODER, BACK_RIGHT_STEER_OFFSET);

                // Update our module positions, odometery, and states
                updateModulePositions();
                updateOdomtery();
                updateDriveStates(states);

                // Zero our gyro
                zeroHeading();

                // Start logging data and adding data to the dashboard
                initLogging();
                initDashboard();

                CommandScheduler.getInstance().registerSubsystem(this);

        }

        @Override
        public void periodic() {
                // Update our module positions, odometery
                updateModulePositions();
                updateOdomtery();
        }

        /**
         * This takes chassis speeds and converts them to module states and then sets states.
         * 
         * @param chassisSpeeds the chassis speeds to convert to module states
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
                if (states != null && chassisSpeeds.vxMetersPerSecond == 0
                                && chassisSpeeds.vyMetersPerSecond == 0
                                && chassisSpeeds.omegaRadiansPerSecond == 0) {
                        states[0] = new SwerveModuleState(0, new Rotation2d(
                                        DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE));
                        states[1] = new SwerveModuleState(0, new Rotation2d(
                                        DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE));
                        states[2] = new SwerveModuleState(0, new Rotation2d(
                                        DrivetrainConstants.BACK_LEFT_RESTING_ANGLE));
                        states[3] = new SwerveModuleState(0, new Rotation2d(
                                        DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE));

                } else {
                        states = kinematics.toSwerveModuleStates(chassisSpeeds);
                }
                setStates(states);
        }

        /**
         * This takes a list of module states and sets them to the modules.
         * 
         * @param states the list of module states to set
         */
        public void updateDriveStates(SwerveModuleState[] states) {
                if (states != null) {
                        SwerveModuleState frontLeftState = states[0];
                        SwerveModuleState frontRightState = states[1];
                        SwerveModuleState backLeftState = states[2];
                        SwerveModuleState backRightState = states[3];

                        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                                        DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

                        frontLeftModule.set(
                                        velocityToDriveVolts(frontLeftState.speedMetersPerSecond),
                                        frontLeftState.angle.getRadians());
                        frontRightModule.set(
                                        velocityToDriveVolts(frontRightState.speedMetersPerSecond),
                                        frontRightState.angle.getRadians());
                        backLeftModule.set(velocityToDriveVolts(backLeftState.speedMetersPerSecond),
                                        backLeftState.angle.getRadians());
                        backRightModule.set(
                                        velocityToDriveVolts(backRightState.speedMetersPerSecond),
                                        backRightState.angle.getRadians());
                }
        }


        /**
         * Method to set states of modules.
         */
        public void setStates(SwerveModuleState[] newStates) {
                states = newStates;
                updateModulePositions();
                updateOdomtery();
                updateDriveStates(states);

        }

        /**
         * Converts a velocity in meters per second to a voltage for the drive motors using
         * feedforward.
         * 
         * @param speedMetersPerSecond the velocity to convert
         * 
         * @return the clamped voltage to apply to the drive motors
         */
        public double velocityToDriveVolts(double speedMetersPerSecond) {
                double ff = feedForward.calculate(speedMetersPerSecond);
                return MathUtil.clamp(ff, -DrivetrainConstants.MAX_VOLTAGE,
                                DrivetrainConstants.MAX_VOLTAGE);
        }

        /**
         * Updates odometry using the current yaw and module states.
         */
        public void updateOdomtery() {
                pose = odometry.update(getHeading2d(), modulePositions);
        }

        /**
         * Updates the module positions array to the current positions of each module
         */
        public void updateModulePositions() {
                modulePositions[0] = frontLeftModule.getPosition();
                modulePositions[1] = frontRightModule.getPosition();
                modulePositions[2] = backLeftModule.getPosition();
                modulePositions[3] = backRightModule.getPosition();
        }

        /**
         * Method to start logging data.
         */
        public void initLogging() {
                DataLogger.addDataElement("fl steer angle",
                                () -> Math.toDegrees(frontLeftModule.getSteerAngle()));
                DataLogger.addDataElement("fl drive velocity",
                                () -> frontLeftModule.getDriveVelocity());
                DataLogger.addDataElement("fr steer angle",
                                () -> Math.toDegrees(frontRightModule.getSteerAngle()));
                DataLogger.addDataElement("fr drive velocity",
                                () -> frontRightModule.getDriveVelocity());
                DataLogger.addDataElement("bl steer angle",
                                () -> Math.toDegrees(backLeftModule.getSteerAngle()));
                DataLogger.addDataElement("bl drive velocity",
                                () -> backLeftModule.getDriveVelocity());
                DataLogger.addDataElement("br steer angle",
                                () -> Math.toDegrees(backRightModule.getSteerAngle()));
                DataLogger.addDataElement("br drive velocity",
                                () -> backRightModule.getDriveVelocity());

                DataLogger.addDataElement("fl target angle", () -> states[0].angle.getDegrees());
                DataLogger.addDataElement("fl target velocity",
                                () -> states[0].speedMetersPerSecond);
                DataLogger.addDataElement("fr target angle", () -> states[1].angle.getDegrees());
                DataLogger.addDataElement("fr target velocity",
                                () -> states[1].speedMetersPerSecond);
                DataLogger.addDataElement("bl target angle", () -> states[2].angle.getDegrees());
                DataLogger.addDataElement("bl target velocity",
                                () -> states[2].speedMetersPerSecond);
                DataLogger.addDataElement("br target angle", () -> states[3].angle.getDegrees());
                DataLogger.addDataElement("br target velocity",
                                () -> states[3].speedMetersPerSecond);

                DataLogger.addDataElement("Heading", () -> getHeading2d().getDegrees());

                DataLogger.addDataElement("poseX", () -> getPose().getX());
                DataLogger.addDataElement("poseY", () -> getPose().getY());
        }

        /**
         * Method to start sending values to the dashboard
         */
        private void initDashboard() {
                tab.addDouble("fl angle", () -> frontLeftModule.getSteerAngle());
                tab.addDouble("fr angle", () -> frontRightModule.getSteerAngle());
                tab.addDouble("bl angle", () -> backLeftModule.getSteerAngle());
                tab.addDouble("br angle", () -> backRightModule.getSteerAngle());

                tab.addDouble("target fl angle", () -> states[0].angle.getDegrees());
                tab.addDouble("target fr angle", () -> states[1].angle.getDegrees());
                tab.addDouble("target bl angle", () -> states[2].angle.getDegrees());
                tab.addDouble("target br angle", () -> states[3].angle.getDegrees());

                tab.addDouble("fl drive vel", () -> frontLeftModule.getDriveVelocity());
                tab.addDouble("bl drive vel", () -> frontLeftModule.getDriveVelocity());
                tab.addDouble("fr drive vel", () -> frontLeftModule.getDriveVelocity());
                tab.addDouble("br drive vel", () -> frontLeftModule.getDriveVelocity());

                tab.addDouble("heading", () -> getHeading2d().getDegrees());
        }

        /**
         * Sets initial pose of robot in meters.
         * 
         * @param initalPosition the initial position of the robot
         * @param initalRotation the initial rotation(heading) of the robot
         */
        public void setInitialPose(Pose2d initalPosition, Rotation2d initalRotation) {
                pigeon.setYaw(initalRotation.getDegrees());
                pose = new Pose2d(initalPosition.getTranslation(), initalRotation);
                odometry = new SwerveDriveOdometry(kinematics, getHeading2d(), modulePositions,
                                pose);

        }

        /**
         * Gets the current pose of the robot.
         * 
         * @return the current pose of the robot in meters
         */
        public Rotation2d getHeading2d() {
                return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
        }

        /**
         * Gets current state of module.
         * 
         * @return the current state of the specified module
         */
        public SwerveModuleState stateFromModule(SwerveModule swerveModule) {
                return new SwerveModuleState(swerveModule.getDriveVelocity(),
                                new Rotation2d(swerveModule.getSteerAngle()));
        }

        /**
         * Converts percent output of joystick to a velocity in meters per second.
         * 
         * @param percentOutput the percent output of the joystick
         * 
         * @return the velocity in meters per second
         */
        public double percentOutputToMetersPerSecond(double percentOutput) {
                return percentOutput * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }

        /**
         * Converts percent output of joystick to a rotational velocity in omega radians per second.
         * 
         * @param percentOutput the percent output of the joystick
         * 
         * @return
         */
        public double percentOutputToRadiansPerSecond(double percentOutput) {
                return percentOutput * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        }

        /**
         * Zeroes the yaw of the pigeon.
         */
        public void zeroHeading() {
                pigeon.setYaw(0);
        }

        /**
         * Gets the current pose of the robot.
         */
        public Pose2d getPose() {
                return pose;
        }

        /**
         * Resets the odometry to the specified pose.
         * 
         * @param pose the pose to which to set the odometry
         */
        public void resetOdometry(Pose2d pose) {
                odometry.resetPosition(getHeading2d(), modulePositions, pose);
        }

        /**
         * Gets the kinematics of the robot.
         * 
         * @return the kinematics of the robot
         */
        public SwerveDriveKinematics getDriveKinematics() {
                return kinematics;
        }

        /**
         * Gets the states of the modules.
         * 
         * @return the states of the modules
         */
        public SwerveModuleState[] getStates() {
                return states;
        }

        /**
         * Gets the front left module.
         * 
         * @return the front left module
         */
        public SwerveModule getFrontLeftModule() {
                return frontLeftModule;
        }

        /**
         * Gets the front right module.
         * 
         * @return the front right module
         */
        public SwerveModule getFrontRightModule() {
                return frontRightModule;
        }

        /**
         * Gets the back left module.
         * 
         * @return the back left module
         */
        public SwerveModule getBackLeftModule() {
                return backLeftModule;
        }

        /**
         * Gets the back right module.
         * 
         * @return the back right module
         */
        public SwerveModule getBackRightModule() {
                return backRightModule;
        }

        /**
         * Gets the chassis speeds.
         * 
         * @return the chassis speeds
         */
        public ChassisSpeeds getChassisSpeeds() {
                return chassisSpeeds;
        }

        /**
         * Sets the chassis speeds.
         * 
         * @param chassisSpeeds the chassis speeds to set
         */
        public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        /**
         * Sets all motor speeds to 0 and sets the modules to their respective resting angles
         */
        public void stop() {
                states[0] = new SwerveModuleState(0,
                                new Rotation2d(DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE));
                states[1] = new SwerveModuleState(0,
                                new Rotation2d(DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE));
                states[2] = new SwerveModuleState(0,
                                new Rotation2d(DrivetrainConstants.BACK_LEFT_RESTING_ANGLE));
                states[3] = new SwerveModuleState(0,
                                new Rotation2d(DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE));
        }
}
