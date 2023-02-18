package frc.robot.subsystems;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.playingwithfusion.TimeOfFlight;
import frc.thunder.swervelib.Mk4ModuleConfiguration;
import frc.thunder.swervelib.Mk4iSwerveModuleHelper;
import frc.thunder.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Offsets;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.HeadingGains;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.logging.DataLogger;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;
import frc.thunder.shuffleboard.LightningShuffleboard;

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
    private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    // Creating new pose, odometry, cahssis speeds
    private Pose2d pose = new Pose2d();
    private SwerveDriveOdometry odometry =
            new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions, pose);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public double FRONT_LEFT_STEER_OFFSET = Offsets.Gridlock.FRONT_LEFT_STEER_OFFSET;
    public double BACK_LEFT_STEER_OFFSET = Offsets.Gridlock.BACK_LEFT_STEER_OFFSET;
    public double FRONT_RIGHT_STEER_OFFSET = Offsets.Gridlock.FRONT_RIGHT_STEER_OFFSET;
    public double BACK_RIGHT_STEER_OFFSET = Offsets.Gridlock.BACK_RIGHT_STEER_OFFSET;

    Path gridlockFile = Paths.get("home/lvuser/gridlock");
    Path blackoutFile = Paths.get("home/lvuser/blackout");

    // Creates our drivetrain shuffleboard tab for displaying module data
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    private final Mk4ModuleConfiguration swerveConfiguration = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration blSwerveConfiguration = new Mk4ModuleConfiguration();
    private Vision vision = new Vision();

    private TimeOfFlight tof = new TimeOfFlight(RobotMap.CAN.TIME_OF_FLIGHT);

    private final PIDController headingController =
            new PIDController(HeadingGains.kP, HeadingGains.kI, HeadingGains.kD);

    private boolean updatedHeading = false;
    private double lastGoodheading = 0d;
    private ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds();

    public Drivetrain(Vision vision) {
        this.vision = vision;

        if (Files.exists(blackoutFile)) {
            FRONT_LEFT_STEER_OFFSET = Offsets.Blackout.FRONT_LEFT_STEER_OFFSET;
            FRONT_RIGHT_STEER_OFFSET = Offsets.Blackout.FRONT_RIGHT_STEER_OFFSET;
            BACK_LEFT_STEER_OFFSET = Offsets.Blackout.BACK_LEFT_STEER_OFFSET;
            BACK_RIGHT_STEER_OFFSET = Offsets.Blackout.BACK_RIGHT_STEER_OFFSET;
        }

        // Set our neo module configurations using drive current, steer current, and
        // voltage
        swerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        swerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        swerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        swerveConfiguration
                .setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        blSwerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        blSwerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        blSwerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        blSwerveConfiguration
                .setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

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
                blSwerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR, RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_LEFT_CANCODER, BACK_LEFT_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                swerveConfiguration, Mk4iSwerveModuleHelper.GearRatio.L2,
                RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR, RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_RIGHT_CANCODER, BACK_RIGHT_STEER_OFFSET);

        // Setting states of the modules
        setStates(states);

        // Update our module positions, odometery, and states
        updateModulePositions();
        updateOdomtery();
        updateDriveStates(states);

        // Zero our gyro
        zeroHeading();

        // Start logging data and adding data to the dashboard
        initLogging();
        initDashboard();

        // PIDDashboardTuner pidTuner = new PIDDashboardTuner("Heading", headingController);

        /*
         * //display gravity vector for PID tuning - leave commented out until tuning neccessary
         * tab.addDouble("gravityX", () -> getGravityVector()[0]); tab.addDouble("gravityY", () ->
         * getGravityVector()[1]); tab.addDouble("gravityZ", () -> getGravityVector()[2]);
         */

        CommandScheduler.getInstance().registerSubsystem(this);

    }

    @Override
    public void periodic() {
        // Update our module positions, odometery
        updateModulePositions();
        updateOdomtery();
        resetOdymetyFVision(getYaw2d(), vision.getRobotPose());

        var board = LightningShuffleboard.getDouble("Autonomous", "pee", 0.015);

        if (headingController.getP() != board) {
            headingController.setP(board);
        }

        // field2d.setRobotPose(pose);
        LightningShuffleboard.setDouble("Autonomous", "Current X", odometry.getPoseMeters().getX());
        LightningShuffleboard.setDouble("Autonomous", "Current Y", odometry.getPoseMeters().getY());
        LightningShuffleboard.setDouble("Autonomous", "Current Z",
                odometry.getPoseMeters().getRotation().getDegrees());
    }

    public PIDController getHeadingController() {
        return headingController;
    }

    public boolean checkModuleStates() {
        return Math.abs(states[0].speedMetersPerSecond + states[1].speedMetersPerSecond
                + states[2].speedMetersPerSecond + states[3].speedMetersPerSecond) == 0;
    }

    /**
     * This takes chassis speeds and converts them to module states and then sets states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        outputChassisSpeeds = chassisSpeeds;

        if (!updatedHeading) {
            lastGoodheading = pose.getRotation().getDegrees();
            updatedHeading = true;
        }

        if (chassisSpeeds.omegaRadiansPerSecond == 0 && !checkModuleStates()) {
            outputChassisSpeeds.omegaRadiansPerSecond =
                    headingController.calculate(pose.getRotation().getDegrees(), lastGoodheading);
        } else {
            updatedHeading = false;
        }

        if (states != null && chassisSpeeds.vxMetersPerSecond == 0
                && chassisSpeeds.vyMetersPerSecond == 0
                && chassisSpeeds.omegaRadiansPerSecond == 0) {

            states[0].speedMetersPerSecond = 0;
            states[1].speedMetersPerSecond = 0;
            states[2].speedMetersPerSecond = 0;
            states[3].speedMetersPerSecond = 0;

        } else {
            states = kinematics.toSwerveModuleStates(outputChassisSpeeds);
        }

        LightningShuffleboard.setDouble("Autonomous", "omegaradpersec",
                outputChassisSpeeds.omegaRadiansPerSecond);

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

            frontLeftModule.set(frontLeftState.speedMetersPerSecond,
                    frontLeftState.angle.getRadians());
            frontRightModule.set(frontRightState.speedMetersPerSecond,
                    frontRightState.angle.getRadians());
            backLeftModule.set(backLeftState.speedMetersPerSecond,
                    backLeftState.angle.getRadians());
            backRightModule.set(backRightState.speedMetersPerSecond,
                    backRightState.angle.getRadians());
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdomtery() {
        pose = odometry.update(getYaw2d(), modulePositions);
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

        DataLogger.addDataElement("fl module position",
                () -> frontLeftModule.getPosition().distanceMeters);
        DataLogger.addDataElement("fr module position",
                () -> frontRightModule.getPosition().distanceMeters);
        DataLogger.addDataElement("bl module position",
                () -> backLeftModule.getPosition().distanceMeters);
        DataLogger.addDataElement("br module position",
                () -> backRightModule.getPosition().distanceMeters);

        DataLogger.addDataElement("fl drive Temperature",
                () -> frontLeftModule.getDriveTemperature());
        DataLogger.addDataElement("fl azimuth Temperature",
                () -> frontLeftModule.getSteerTemperature());
        DataLogger.addDataElement("fr drive Temperature",
                () -> frontRightModule.getDriveTemperature());
        DataLogger.addDataElement("fr azimuth Temperature",
                () -> frontRightModule.getSteerTemperature());
        DataLogger.addDataElement("bl drive Temperature",
                () -> backLeftModule.getDriveTemperature());
        DataLogger.addDataElement("bl azimuth Temperature",
                () -> backLeftModule.getSteerTemperature());
        DataLogger.addDataElement("br drive Temperature",
                () -> backRightModule.getDriveTemperature());
        DataLogger.addDataElement("br azimuth Temperature",
                () -> backRightModule.getSteerTemperature());

        DataLogger.addDataElement("fl target angle", () -> states[0].angle.getDegrees());
        DataLogger.addDataElement("fl target velocity", () -> states[0].speedMetersPerSecond);
        DataLogger.addDataElement("fr target angle", () -> states[1].angle.getDegrees());
        DataLogger.addDataElement("fr target velocity", () -> states[1].speedMetersPerSecond);
        DataLogger.addDataElement("bl target angle", () -> states[2].angle.getDegrees());
        DataLogger.addDataElement("bl target velocity", () -> states[2].speedMetersPerSecond);
        DataLogger.addDataElement("br target angle", () -> states[3].angle.getDegrees());
        DataLogger.addDataElement("br target velocity", () -> states[3].speedMetersPerSecond);

        DataLogger.addDataElement("fl drive voltage", () -> frontLeftModule.getDriveVoltage());
        DataLogger.addDataElement("fr drive voltage", () -> frontRightModule.getDriveVoltage());
        DataLogger.addDataElement("bl drive voltage", () -> backLeftModule.getDriveVoltage());
        DataLogger.addDataElement("br drive voltage", () -> backRightModule.getDriveVoltage());

        DataLogger.addDataElement("Heading",
                () -> odometry.getPoseMeters().getRotation().getDegrees());
        DataLogger.addDataElement("poseX", () -> odometry.getPoseMeters().getX());
        DataLogger.addDataElement("poseY", () -> odometry.getPoseMeters().getY());

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

        tab.addDouble("heading", () -> getYaw2d().getDegrees());
        tab.addDouble("roll", () -> getRoll2d().getDegrees());
        tab.addDouble("pitch", () -> getPitch2d().getDegrees());

        tab.addDouble("fl drive voltage", () -> frontLeftModule.getDriveVoltage());
    }

    public PathPoint getCurrentPathPoint() {
        return new PathPoint(
                new Translation2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()),
                odometry.getPoseMeters().getRotation());
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

    public Rotation2d getHeading() {
        return odometry.getPoseMeters().getRotation();
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return the current heading of the robot in meters
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
    }

    /**
     * Gets the current pitch of the robot.
     * 
     * @return the current pitch of the robot in meters
     */
    public Rotation2d getPitch2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getPitch(), -180, 180));
    }

    /**
     * Gets the current roll of the robot.
     * 
     * @return the current roll of the robot in meters
     */
    public Rotation2d getRoll2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getRoll(), -180, 180));
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
        odometry.resetPosition(getYaw2d(), modulePositions, pose);
    }

    /**
     * Takes pose2d from vision and resets odometry to that pose. Sets module positions to the
     * current module positions.
     * 
     * @param gyroAngle the current yaw of the robot
     * @param pose the pose from Vision of the robot
     */
    public void resetOdymetyFVision(Rotation2d gyroAngle, Pose2d pose) {
        // if (pose != null) {
        // odometry.resetPosition(gyroAngle, modulePositions, pose);
        // }
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
        states[0].speedMetersPerSecond = 0;
        states[1].speedMetersPerSecond = 0;
        states[2].speedMetersPerSecond = 0;
        states[3].speedMetersPerSecond = 0;
        // states[0] = new SwerveModuleState(0,
        // new Rotation2d(DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE));
        // states[1] = new SwerveModuleState(0,
        // new Rotation2d(DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE));
        // states[2] = new SwerveModuleState(0,
        // new Rotation2d(DrivetrainConstants.BACK_LEFT_RESTING_ANGLE));
        // states[3] = new SwerveModuleState(0,
        // new Rotation2d(DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE));
    }

    public void resetNeoAngle() {
        frontLeftModule.setEncoderAngle();
        frontRightModule.setEncoderAngle();
        backLeftModule.setEncoderAngle();
        backRightModule.setEncoderAngle();
    }

    public double getTOF() {
        return tof.getRange();
    }
}
