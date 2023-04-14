package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.accessibility.AccessibleHyperlink;

import org.apache.commons.lang3.Range;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.thunder.swervelib.Mk4ModuleConfiguration;
import frc.thunder.swervelib.Mk4iSwerveModuleHelper;
import frc.thunder.swervelib.SwerveModule;
import frc.thunder.vision.VisionBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Offsets;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.HeadingGains;
import frc.thunder.LightningRobot;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;
import frc.thunder.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {

    // Creates our swerve kinematics using the robots track width and wheel base
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    //Target to focus on
    private int aprilTagTarget = -1;

    // Creating new pigeon2 IMU
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.CAN.PIGEON_ID);

    // Creating our list of module states and module positions
    private SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    // Creating new pose, odometry, cahssis speeds
    private Pose2d pose = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // Module steer offsets
    private double FRONT_LEFT_STEER_OFFSET = Offsets.Gridlock.FRONT_LEFT_STEER_OFFSET;
    private double BACK_LEFT_STEER_OFFSET = Offsets.Gridlock.BACK_LEFT_STEER_OFFSET;
    private double FRONT_RIGHT_STEER_OFFSET = Offsets.Gridlock.FRONT_RIGHT_STEER_OFFSET;
    private double BACK_RIGHT_STEER_OFFSET = Offsets.Gridlock.BACK_RIGHT_STEER_OFFSET;

    // Swerve pose esitmator for odometry
    SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, getYaw2d(), modulePositions, new Pose2d(), DrivetrainConstants.STANDARD_DEV_POSE_MATRIX, VisionConstants.STANDARD_DEV_VISION_MATRIX);
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions);

    // Creates our drivetrain shuffleboard tab for displaying module data and a periodic shuffleboard for data that doesn't need constant updates
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    private LightningShuffleboardPeriodic periodicShuffleboard;
    private LightningShuffleboardPeriodic periodicShuffleboardAuto;
    private final Mk4ModuleConfiguration swerveConfiguration = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration blSwerveConfiguration = new Mk4ModuleConfiguration();

    // PIDController for heading compenstation
    private final PIDController headingController = new PIDController(HeadingGains.kP, HeadingGains.kI, HeadingGains.kD);

    private boolean flipFL = false;
    private boolean flipFR = false;
    private boolean flipBR = false;
    private boolean flipBL = false;

    private boolean doVisionUpdate = true;
    private double lastKnownGoodVisionX = 0;
    private double lastKnownGoodVisionY = 0;
    private double lastTime = 0;

    // Heading compenstaion variables
    // private boolean updatedHeading = false;
    // private double lastGoodheading = 0d;

    // Chassis speeds for the robot
    private ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds();

    private LimelightBack limelightBack;
    private LimelightFront limelightFront;

    // Manual trajectory variables
    private Pose2d desiredPose = new Pose2d();
    private double maxVel = 0d;
    private double maxAccell = 0d;

    private double velocity = 0;

    private Pose2d visionPose2d;

    public Drivetrain(LimelightBack limelightBack, LimelightFront limelightFront) {
        this.limelightBack = limelightBack;
        this.limelightFront = limelightFront;
        /**
         * Creates a new Drivetrain.
         * 
         * @param vision the vision subsystem
         */

        if (Constants.isBlackout()) {
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
        swerveConfiguration.setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        blSwerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        blSwerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        blSwerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        blSwerveConfiguration.setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        // Making front left module
        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR, RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_LEFT_CANCODER, FRONT_LEFT_STEER_OFFSET);

        // Making front right module
        frontRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR, RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_RIGHT_CANCODER, FRONT_RIGHT_STEER_OFFSET);

        // Making backleft module
        backLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), blSwerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR, RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_LEFT_CANCODER, BACK_LEFT_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR, RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_RIGHT_CANCODER, BACK_RIGHT_STEER_OFFSET);

        // Setting start position and creating estimator
        setInitialPose(new Pose2d(0, 0, new Rotation2d()));

        // Setting states of the modules
        states = new SwerveModuleState[] {new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontLeftModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontRightModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), backLeftModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), backRightModule.getPosition().angle)};
        updateOdometry();
        updateDriveStates(states);

        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();

        // Zero our gyro
        zeroHeading();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {

        if(DriverStation.isDisabled()) {
            states = new SwerveModuleState[] {new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontLeftModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), frontRightModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), backLeftModule.getPosition().angle), new SwerveModuleState(frontLeftModule.getDriveVelocity(), backRightModule.getPosition().angle)};
        }

        // Update our odometry
        updateOdometry();
        updateVision();

        periodicShuffleboard.loop();
        periodicShuffleboardAuto.loop();
    }

    public double getDriveVelocity() {
        return (frontLeftModule.getDriveVelocity() + frontRightModule.getDriveVelocity() + backLeftModule.getDriveVelocity() + backRightModule.getDriveVelocity()) / 4;
    }

    /**
     * Method to return the heading controller
     * 
     * @return the heading PIDController
     */
    public PIDController getHeadingController() {
        return headingController;
    }

    public void hardResetToVision() {
        if(visionPose2d != null) {
            poseEstimator.resetPosition(getHeading(), modulePositions, visionPose2d);
        }
    }

    /**
     * This takes chassis speeds and converts them to module states and then sets states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        outputChassisSpeeds = chassisSpeeds;

        // If were not commanding any thing to the motors, make sure our states speeds are 0
        if (states != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
            states[0].speedMetersPerSecond = 0;
            states[1].speedMetersPerSecond = 0;
            states[2].speedMetersPerSecond = 0;
            states[3].speedMetersPerSecond = 0;

        } else {
            // If not set the states to the commanded chassis speeds
            states = kinematics.toSwerveModuleStates(outputChassisSpeeds);
        }

        if (flipFL) {
            states[0].speedMetersPerSecond *= -1;
        }
        if (flipFR) {
            states[1].speedMetersPerSecond *= -1;
        }
        if (flipBL) {
            states[2].speedMetersPerSecond *= -1;
        }
        if (flipBR) {
            states[3].speedMetersPerSecond *= -1;
        }

        // Sets the states to the modules
        setStates(states);
    }

    public ChassisSpeeds getOutputChassisSpeeds() {
        return outputChassisSpeeds;
    }

    public void flipFL() {
        flipFL = !flipFL;
    }

    public void flipFR() {
        flipFR = !flipFR;
    }

    public void flipBL() {
        flipBL = !flipBL;
    }

    public void flipBR() {
        flipBR = !flipBR;
    }

    public Rotation2d getDriveHeading(double xMeters, double yMeters) {
        double changeX = pose.getX() - xMeters;
        double changeY = pose.getY() - yMeters;

        return Rotation2d.fromRadians(Math.atan2(changeY, changeX));
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

            // Normalize the wheel speeds if the magnitude of any wheel is greater than max velocity
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

            // Sets the states to the modules
            frontLeftModule.set(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getRadians());
            frontRightModule.set(frontRightState.speedMetersPerSecond, frontRightState.angle.getRadians());
            backLeftModule.set(backLeftState.speedMetersPerSecond, backLeftState.angle.getRadians());
            backRightModule.set(backRightState.speedMetersPerSecond, backRightState.angle.getRadians());
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdometry() {
        updateModulePositions();
        pose = poseEstimator.update(getYaw2d(), modulePositions);
        rawPose = odometry.update(getYaw2d(), modulePositions);

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //     pose = new Pose2d(16.48 - pose.getX(), pose.getY(), pose.getRotation());
        // }
    }

    private boolean firstTime = true;

    public void updateVision() {
        if (VisionBase.isVisionEnabled()) {
            visionPose2d = null;
            double latency = 0;
            double tagDistance = 0;
            if (limelightFront.hasVision()) {
                visionPose2d = limelightFront.getRobotPose();
                latency = limelightFront.getLatencyBotPoseBlue();
                tagDistance = limelightFront.getTagDistance();
                if (firstTime) {
                    setInitialPose(visionPose2d);
                    firstTime = false;
                }
            } else if (limelightBack.hasVision()) {
                visionPose2d = limelightBack.getRobotPose();
                latency = limelightBack.getLatencyBotPoseBlue();
                tagDistance = limelightBack.getTagDistance();
                if (firstTime) {
                    setInitialPose(visionPose2d);

                    firstTime = false;
                }
            }

            if (visionPose2d == null || visionPose2d.getX() > 23.04 || visionPose2d.getY() > 8.02 || visionPose2d.getX() < 0 || visionPose2d.getY() < 0 || tagDistance > 4) {
                return;
            }

            double currTime = Timer.getFPGATimestamp();
            LightningShuffleboard.setDouble("Drivetrain", "Velocity between points", pose.getTranslation().getDistance(visionPose2d.getTranslation()) / (currTime - lastTime));
            if (pose.getTranslation().getDistance(visionPose2d.getTranslation()) / (currTime - lastTime) > DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND) {
                return;
            }

            // if (tagDistance != -1) {
            //     double distanceBasedDev = VisionConstants.visionStandardDevMap.get(tagDistance + getDriveVelocity());
            //     poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(distanceBasedDev, distanceBasedDev, distanceBasedDev));
            // }

            poseEstimator.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp() - latency - .0472);
            pose = poseEstimator.getEstimatedPosition();

            lastKnownGoodVisionX = visionPose2d.getX();
            lastKnownGoodVisionY = visionPose2d.getY();
            lastTime = currTime;

            LightningShuffleboard.setDouble("Drivetrain", "Accepted vision X", lastKnownGoodVisionX);
            LightningShuffleboard.setDouble("Drivetrain", "Accepted vision Y", lastKnownGoodVisionY);
        }
    }

    /**
     * ta
     * 
     * @param pos Pos of tag 1 at C nodes
     */
    public void setAprilTagTarget(int pos) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            pos += 6;
        } else {
            pos += 3;
        }
        limelightBack.setPipelineNum(pos);
        limelightFront.setPipelineNum(pos);
    }

    public void setAprilTagTargetAll() {
        limelightBack.setPipelineNum(0);
        limelightFront.setPipelineNum(0);
    }

    /**
     * 2 Method to set states of modules.
     */
    public void setStates(SwerveModuleState[] newStates) {
        states = newStates;
        updateOdometry();
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

    // Method to start sending values to the dashboard and start logging
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Drivetrain", DrivetrainConstants.LOG_PERIOD, new Pair<String, Object>("Pigeon Yaw", (DoubleSupplier) () -> getYaw2d().getDegrees()),
                new Pair<String, Object>("roll", (DoubleSupplier) () -> pigeon.getRoll()), new Pair<String, Object>("pitch", (DoubleSupplier) () -> pigeon.getPitch()),
                new Pair<String, Object>("fl module position", (DoubleSupplier) () -> modulePositions[0].distanceMeters),
                new Pair<String, Object>("fr module position", (DoubleSupplier) () -> modulePositions[1].distanceMeters),
                new Pair<String, Object>("bl module position", (DoubleSupplier) () -> modulePositions[2].distanceMeters),
                new Pair<String, Object>("br module position", (DoubleSupplier) () -> modulePositions[3].distanceMeters),
                new Pair<String, Object>("odo Pose", (Supplier<double[]>) () -> new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()}),
                new Pair<String, Object>("raw Pose", (Supplier<double[]>) () -> new double[] {rawPose.getX(), rawPose.getY(), rawPose.getRotation().getRadians()}),
                new Pair<String, Object>("desired X", (DoubleSupplier) () -> desiredPose.getX()), new Pair<String, Object>("desired Y", (DoubleSupplier) () -> desiredPose.getY()),
                new Pair<String, Object>("desired Z", (DoubleSupplier) () -> desiredPose.getRotation().getDegrees()));

        periodicShuffleboardAuto = new LightningShuffleboardPeriodic("Autonomous", new Pair<String, Object>("has vision", (BooleanSupplier) () -> limelightBack.hasVision()),
                new Pair<String, Object>("Vison GOOD", (BooleanSupplier) () -> !firstTime));
    }

    /**
     * Gets the current pathplanner path point of the robot in meters using
     */
    public PathPoint getCurrentPathPoint() {
        return PathPoint.fromCurrentHolonomicState(pose, chassisSpeeds).withControlLengths(AutoAlignConstants.CONTROL_LENGTHS, AutoAlignConstants.CONTROL_LENGTHS);
    }

    public void instantSyncVision() {
        if (visionPose2d != null) {
            pose = visionPose2d;
            System.out.println("Ran the Sync");
        }
    }

    /**
     * Sets initial pose of robot in meters.
     * 
     * @param initalPosition the initial position of the robot
     */
    public void setInitialPose(Pose2d initalPosition) {
        pigeon.setYaw(initalPosition.getRotation().getDegrees());
        pose = new Pose2d(initalPosition.getTranslation(), initalPosition.getRotation());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw2d(), modulePositions, pose);
    }

    /**
     * Gets the heading of the robot from odometry in rotation 2d
     */
    public Rotation2d getHeading() {
        return pose.getRotation();
    }

    /**
     * Gets the current rotation from the pigeon
     * 
     * @return the current heading of the robot in degrees from 0 to 360
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
    }

    /**
     * Gets the current pitch of the robot from the pigeon
     * 
     * @return the current pitch of the robot in degrees from -180 to 180
     */
    public Rotation2d getPitch2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getPitch(), -180, 180));
    }

    /**
     * Gets the current roll of the robot from the pigeon
     * 
     * @return the current roll of the robot in degrees from -180 to 180
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
        return new SwerveModuleState(swerveModule.getDriveVelocity(), new Rotation2d(swerveModule.getSteerAngle()));
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

    public void setYaw(double input) {
        pigeon.setYaw(input);
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
        poseEstimator.resetPosition(getYaw2d(), modulePositions, pose);
    }

    public void poseReset(Pose2d pose) {
        poseEstimator.resetPosition(getYaw2d(), modulePositions, pose);
        odometry.resetPosition(getHeading(), modulePositions, pose);
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
     * Gets if it is in community
     * 
     * @return if in community or not
     */
    public boolean isInCommunity() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            if ((1.35 < pose.getX() && pose.getX() < 3.35) && (1.50 < pose.getY() && pose.getY() < 5.25) || // Box 1
                    (1.35 < pose.getX() && pose.getX() < 4.85) && (0.00 < pose.getY() && pose.getY() < 1.50) || // Box 2
                    (9.85 < pose.getX() && pose.getX() < 13.20) && (6.78 < pose.getY() && pose.getY() < 7.99) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (5.51 < pose.getY() && pose.getY() < 7.99)) { // Box 4
                return true;
            }
        } else if (DriverStation.getAlliance() == Alliance.Red) { // Check if I did it right on field
            if ((1.35 < pose.getX() && pose.getX() < 3.35) && (2.40 < pose.getY() && pose.getY() < 6.45) || // Box 1
                    (1.35 < pose.getX() && pose.getX() < 4.85) && (6.45 < pose.getY() && pose.getY() < 8.02) || // Box 2
                    (9.85 < pose.getX() && pose.getX() < 13.20) && (0.03 < pose.getY() && pose.getY() < 1.24) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (0.03 < pose.getY() && pose.getY() < 2.51)) { // Box 4
                return true;
            }
        }
        return false;
    }

    public boolean isInLoadZone() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            if ((9.85 < pose.getX() && pose.getX() < 13.20) && (6.78 < pose.getY() && pose.getY() < 7.99) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (5.51 < pose.getY() && pose.getY() < 7.99)) { // Box 4
                return true;
            }
        } else if (DriverStation.getAlliance() == Alliance.Red) { // Check if I did it right on field
            if ((9.85 < pose.getX() && pose.getX() < 13.20) && (0.03 < pose.getY() && pose.getY() < 1.24) || // Box 3
                    (13.20 < pose.getX() && pose.getX() < 16.25) && (0.03 < pose.getY() && pose.getY() < 2.51)) { // Box 4
                return true;
            }
        }
        return false;
    }

    /**
     * Sets all motor speeds to 0 and sets the modules to their respective resting angles
     */
    public void stop() {
        frontLeftModule.set(0, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE);
        frontRightModule.set(0, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE);
        backLeftModule.set(0, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE);
        backRightModule.set(0, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE);

    }

    public void resetNeoAngle() {
        frontLeftModule.setEncoderAngle();
        frontRightModule.setEncoderAngle();
        backLeftModule.setEncoderAngle();
        backRightModule.setEncoderAngle();
    }

    public void setDesiredPose(Pose2d desiredPose) {
        this.desiredPose = desiredPose;

    }

    public Pose2d getDesiredPose() {
        return desiredPose;
    }

    public void moveToDesiredPose(AutonomousCommandFactory autoFactory) {

        maxVel = MathUtil.clamp(pose.getTranslation().getDistance(desiredPose.getTranslation()) * 2, 0.5, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
        maxAccell = 5;

        System.out.println("desired X: " + desiredPose.getX());
        System.out.println("desired Y: " + desiredPose.getX());
        System.out.println("odo X: " + pose.getX());
        System.out.println("odo Y: " + pose.getY());

        // MathUtil.clamp(pose.getTranslation().getDistance(desiredPose.getTranslation()) / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        //         -DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

        Rotation2d driveHeading = getDriveHeading(desiredPose.getX(), desiredPose.getY());

        autoFactory.createManualTrajectory(new PathConstraints(maxVel, maxAccell), getCurrentPathPoint(),
                new PathPoint(desiredPose.getTranslation(), driveHeading, desiredPose.getRotation()).withControlLengths(AutoAlignConstants.CONTROL_LENGTHS, AutoAlignConstants.CONTROL_LENGTHS));

    }

    public boolean onTarget() {
        return Math.abs(pose.getTranslation().getDistance(desiredPose.getTranslation())) < 0.1;
    }

    // public void getPoseFromIMU() {
    //     short[] xyz = new short[3];
	// 	pigeon.getBiasedAccelerometer(xyz);
	// 	Translation3d transVector =  new Translation3d(xyz[0] / 16384d * 9.81d, xyz[1] / 16384d * 9.81d, xyz[2] / 16384d * 9.81d);
    // }
}
