package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.logging.DataLogger;
import frc.thunder.tuning.PIDDashboardTuner;

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

    // Creating new pose, odometry, and cahssis speeds
    private Pose2d pose = new Pose2d();
    private SwerveModulePosition[] modulePositions = { new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions, pose);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our list of module states
    private SwerveModuleState[] states = { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState() };

    // Creating our modules
    private final NeoSwerveModule frontLeftModule;
    private final NeoSwerveModule frontRightModule;
    private final NeoSwerveModule backLeftModule;
    private final NeoSwerveModule backRightModule;

    private final GenericEntry angle;

    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    public Drivetrain() {

        // Making front left module
        frontLeftModule = new NeoSwerveModule(RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR,
                RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_LEFT_CANCODER,
                DrivetrainConstants.FRONT_LEFT_STEER_OFFSET, "a");

        // Making front right module
        frontRightModule = new NeoSwerveModule(RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR,
                RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_RIGHT_CANCODER,
                DrivetrainConstants.FRONT_RIGHT_STEER_OFFSET, "b");

        // Making backleft module
        backLeftModule = new NeoSwerveModule(RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR,
                RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_LEFT_CANCODER,
                DrivetrainConstants.BACK_LEFT_STEER_OFFSET, "c");

        // Making back right module
        backRightModule = new NeoSwerveModule(RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR,
                RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_RIGHT_CANCODER,
                DrivetrainConstants.BACK_RIGHT_STEER_OFFSET, "d");

        // Update our module positions
        updateModulePositions();

        // Zero our gyro
        zeroYaw();

        // Start logging data
        initLogging();
        initDashboard();

        DrivetrainConstants.AZIMUTH_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        DrivetrainConstants.AZIMUTH_PID_CONTROLLER.setTolerance(0.09);

        PIDDashboardTuner drivePidDashboardTuner = new PIDDashboardTuner("drive",
                DrivetrainConstants.DRIVE_PID_CONTROLLER);
        PIDDashboardTuner azimuthPidDashboardTuner = new PIDDashboardTuner("azimuth",
                DrivetrainConstants.AZIMUTH_PID_CONTROLLER);

        this.angle = tab.add("angle set", 0).getEntry();

        CommandScheduler.getInstance().registerSubsystem(this);

    }

    @Override
    public void periodic() {
        // Update our module positions, odometery
        updateModulePositions();
        updateOdomtery();
    }

    /**
     * This takes chassis speeds and converts them to module states and then sets
     * states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        if (states != null && chassisSpeeds.vxMetersPerSecond == 0
                && chassisSpeeds.vyMetersPerSecond == 0
                && chassisSpeeds.omegaRadiansPerSecond == 0) {
            // states[0] = new SwerveModuleState(0,
            // new Rotation2d(DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE));
            // states[1] = new SwerveModuleState(0,
            // new Rotation2d(DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE));
            // states[2] = new SwerveModuleState(0,
            // new Rotation2d(DrivetrainConstants.BACK_LEFT_RESTING_ANGLE));
            // states[3] = new SwerveModuleState(0,
            // new Rotation2d(DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE));

            states[0].speedMetersPerSecond = 0;
            states[1].speedMetersPerSecond = 0;
            states[2].speedMetersPerSecond = 0;
            states[3].speedMetersPerSecond = 0;

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
    public void setStates(SwerveModuleState[] states) {
        this.states = states;
        updateModulePositions();
        updateOdomtery();

        if (states != null) {
            SwerveModuleState frontLeftState = states[0];
            SwerveModuleState frontRightState = states[1];
            SwerveModuleState backLeftState = states[2];
            SwerveModuleState backRightState = states[3];

            SwerveDriveKinematics.desaturateWheelSpeeds(states,
                    DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

            frontLeftModule.setDesiredState(frontLeftState);
            frontRightModule.setDesiredState(frontRightState);
            backLeftModule.setDesiredState(backLeftState);
            backRightModule.setDesiredState(backRightState);
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdomtery() {
        pose = odometry.update(getYaw2d(), modulePositions);
    }

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
        DataLogger.addDataElement("fl drive velocity", () -> frontLeftModule.getDriveVelocity());
        DataLogger.addDataElement("fr steer angle",
                () -> Math.toDegrees(frontRightModule.getSteerAngle()));
        DataLogger.addDataElement("fr drive velocity", () -> frontRightModule.getDriveVelocity());
        DataLogger.addDataElement("bl steer angle",
                () -> Math.toDegrees(backLeftModule.getSteerAngle()));
        DataLogger.addDataElement("bl drive velocity", () -> backLeftModule.getDriveVelocity());
        DataLogger.addDataElement("br steer angle",
                () -> Math.toDegrees(backRightModule.getSteerAngle()));
        DataLogger.addDataElement("br drive velocity", () -> backRightModule.getDriveVelocity());

        DataLogger.addDataElement("Heading", () -> getYaw2d().getDegrees());

        DataLogger.addDataElement("poseX", () -> getPose().getX());
        DataLogger.addDataElement("poseY", () -> getPose().getY());
    }

    public void initDashboard() {
        // Creates our drivetrain shuffleboard tab for displaying module data
        // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // ShuffleboardLayout frontLeft = tab.getLayout("Front Left",
        // BuiltInLayouts.kList).withSize(2, 1)
        // .withPosition(0, 0);

        // ShuffleboardLayout frontRight = tab.getLayout("Front Right",
        // BuiltInLayouts.kList).withSize(2, 1)
        // .withPosition(0, 0);

        // ShuffleboardLayout backLeft = tab.getLayout("Back Left",
        // BuiltInLayouts.kList).withSize(2, 1)
        // .withPosition(0, 0);

        // ShuffleboardLayout backRight = tab.getLayout("Back Right",
        // BuiltInLayouts.kList).withSize(2, 1)
        // .withPosition(0, 0);

        // if (states != null) {
        // frontLeft.add("Current Angle", frontLeftModule.getSteerAngle());
        // frontLeft.add("Target Angle", states[0].angle.getDegrees());
        // frontLeft.add("Current Velocity", frontLeftModule.getDriveVelocity());
        // frontLeft.add("Target Velocity", states[0]);

        // frontRight.add("Current Angle", frontRightModule.getSteerAngle());
        // frontRight.add("Target Angle", states[1].angle.getDegrees());
        // frontRight.add("Current Velocity", frontRightModule.getDriveVelocity());
        // frontRight.add("Target Velocity", states[1]);

        // backLeft.add("Current Angle", backLeftModule.getSteerAngle());
        // backLeft.add("Target Angle", states[2].angle.getDegrees());
        // backLeft.add("Current Velocity", backLeftModule.getDriveVelocity());
        // backLeft.add("Target Velocity", states[2]);

        // backRight.add("Current Angle", backRightModule.getSteerAngle());
        // backRight.add("Target Angle", states[3].angle.getDegrees());
        // backRight.add("Current Velocity", backRightModule.getDriveVelocity());
        // backRight.add("Target Velocity", states[3]);
        // }

        tab.addDouble("fl angle", () -> frontLeftModule.getCancoder().getPosition());
        tab.addDouble("fr angle", () -> frontRightModule.getCancoder().getPosition());
        tab.addDouble("bl angle", () -> backLeftModule.getCancoder().getPosition());
        tab.addDouble("br angle", () -> backRightModule.getCancoder().getPosition());

        tab.addDouble("target fl angle", () -> states[0].angle.getDegrees());
        tab.addDouble("target fr angle", () -> states[1].angle.getDegrees());
        tab.addDouble("target bl angle", () -> states[2].angle.getDegrees());
        tab.addDouble("target br angle", () -> states[3].angle.getDegrees());

        tab.addDouble("pid offset", () -> DrivetrainConstants.DRIVE_PID_CONTROLLER.getPositionError());
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
        odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions, pose);

    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return the current pose of the robot in meters
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
    }

    /**
     * Gets current state of module.
     * 
     * @return the current state of the specified module
     */
    public SwerveModuleState stateFromModule(NeoSwerveModule swerveModule) {
        return swerveModule.getState();
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
     * Converts percent output of joystick to a rotational velocity in omega radians
     * per second.
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
    public void zeroYaw() {
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
        odometry.resetPosition(getYaw2d(), modulePositions, pose);
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
    public NeoSwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    /**
     * Gets the front right module.
     * 
     * @return the front right module
     */
    public NeoSwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    /**
     * Gets the back left module.
     * 
     * @return the back left module
     */
    public NeoSwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    /**
     * Gets the back right module.
     * 
     * @return the back right module
     */
    public NeoSwerveModule getBackRightModule() {
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

    public void stop() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0,
                new Rotation2d(DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE)));
        frontRightModule.setDesiredState(new SwerveModuleState(0,
                new Rotation2d(DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE)));
        backLeftModule.setDesiredState(new SwerveModuleState(0,
                new Rotation2d(DrivetrainConstants.BACK_LEFT_RESTING_ANGLE)));
        backRightModule.setDesiredState(new SwerveModuleState(0,
                new Rotation2d(DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE)));
    }
}
