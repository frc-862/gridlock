package frc.robot;

import java.awt.Polygon;
import java.nio.file.Path;
import java.nio.file.Paths;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.thunder.pathplanner.com.pathplanner.lib.auto.PIDConstants;
import frc.thunder.swervelib.SdsModuleConfigurations;

public final class Constants {

    public static final double VOLTAGE_COMP_VOLTAGE = 12d;

    public static final Path BLACKOUT_FILE = Paths.get("home/lvuser/blackout");

    public static final boolean isBlackout() {
        return BLACKOUT_FILE.toFile().exists();
    }

    public static final boolean isGridlock() {
        return !isBlackout();
    }

    // Constants for xbox controlers
    public static final class XboxControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.05;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 0.9d;

    }

    public static final class SystemTestConstants {
        // Drive Test Variables
        public static final int DEGREES_INTERVAL_INCREASE = 30;
        public static final int ANGLE_DEAD_ZONE = 3;
        public static final int MAX_ROTATIONS_PER_DIRECTION = 2;
    }

    public static final class DrivetrainConstants {

        // Our drivetrain and track width
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.8125d);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.8125d);

        // Module resting/default angles
        public static final double FRONT_LEFT_RESTING_ANGLE = Math.toRadians(-45d);
        public static final double FRONT_RIGHT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_LEFT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_RIGHT_RESTING_ANGLE = Math.toRadians(-45d);

        // Our max voltage, velocity, angular velocity, and angular acceleration
        public static final double MAX_VOLTAGE = 12;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0
                * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                * 2 * Math.PI;

        // Module configuration constants
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int STEER_CURRENT_LIMIT = 30;
        public static final double NOMINAL_VOLTAGE = 12d;

        // // Standard dev for robot pose
        public static final Matrix<N3, N1> STANDARD_DEV_POSE_MATRIX = VecBuilder.fill(0.3313838876, 0.2642363651,
                0.03681853519);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.15;// .116d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kF = 0.225;// 229d;
        }

        // Gains vaules for theta PIDControllers
        public static final class ThetaGains {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;

        }

        public static final class HeadingGains {
            public static final double kP = 0.005d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        public static final class Offsets {
            public static final class Gridlock {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(193.535);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(145.547);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(198.721);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(210.938);
            }

            public static final class Blackout {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(253.916);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(222.451);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(19.688);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(63.018);
            }
        }
    }

    public static final class ElevatorConstants {
        public static final boolean MOTOR_INVERT = false;

        public static final int CURRENT_LIMIT = 40;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        public static final double kP = 0d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kF = 0d;

        // TODO: set a tolerance
        public static final double TOLERANCE = 0d;

        // TOOD: replace with actual values
        public static final int TICKS_PER_REV = 42;
        public static final double GEAR_RATIO = 16d / 1d;
        public static final double SPROCKET_DIAMETER = 1.440d;
        public static final double POSITION_CONVERSION_FACTOR = 1 / GEAR_RATIO * SPROCKET_DIAMETER * Math.PI;

        // min/max height in inches
        public static final double MAX_EXTENSION = 23.287d;
        public static final double MIN_EXTENSION = 0d;

        // the height of the elevator from the ground, use as an offset for our math
        public static final double ELEVATOR_HEIGHT_OFFSET = 40d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final SparkMaxLimitSwitch.Type TOP_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type BOTTOM_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;

        public static final Rotation2d ANGLE = Rotation2d.fromDegrees(55d);
        // Acute Elevator mount angle in degrees

        // horiz/vert offset from ground (See below)
        public static final Translation2d POSE_OFFSET = new Translation2d(0, 0);
        // X = distance from arm pivot point to front of bot at bottom limit (negative)
        // Y = height of arm pivot point from ground at bottom limit
    }

    public static final class ArmConstants {
        public static final boolean MOTOR_INVERT = false;

        public static final int CURRENT_LIMIT = 40;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        public static final double kP = 0d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kF = 0d;

        public static final double TOLERANCE = 0d;

        // Min and Max arm angles in degrees
        public static final double MAX_ANGLE = 90d;
        public static final double MIN_ANGLE = -90d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final double LENGTH = 30; // arm length in inches

        // Offsets in degrees
        public static final double ENCODER_OFFSET_GRIDLOCK = 0;
        public static final double ENCODER_OFFSET_BLACKOUT = 0;

        // robot lengths
        // TODO: get accurate measurements
        public static final double ROBOT_BODY_LENGTH = 27.7;

        // TODO: replace with actual values
        public static final double GEAR_RATIO = 1d;
        public static final double DEGREES_PER_REV = 1d;
        public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO * DEGREES_PER_REV;
    }

    public static final class WristConstants {
        public static final boolean MOTOR_INVERT = false;

        public static final int CURRENT_LIMIT = 20;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        public static final double kP = 0d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kF = 0d;

        public static final double TOLERANCE = 0d;

        // min/max angles in degrees
        public static final double MAX_ANGLE = 90d;
        public static final double MIN_ANGLE = -90d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final Translation2d POSE_OFFSET = new Translation2d(0, 0);

        public static final double LENGTH = 0; // wrist length in inches

        // Offsets in degrees
        public static final double ENCODER_OFFSET_GRIDLOCK = 0;
        public static final double ENCODER_OFFSET_BLACKOUT = 0;

        // TODO: replace with actual values
        public static final double GEAR_RATIO = 1d;
        public static final double DEGREES_PER_REV = 1d;
        public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO * DEGREES_PER_REV;
    }

    public static final class RobotMap {
        public static final class CAN {
            // Pigeon IMU ID
            public static final int PIGEON_ID = 23;
            // Power distrobution hub ID
            public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 2;
            public static final int FRONT_LEFT_CANCODER = 31;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 32;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 33;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 7;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 8;
            public static final int BACK_LEFT_CANCODER = 34;

            // COLLECTOR
            public static final int LEFT_COLLECTOR_MOTOR = 12;
            public static final int RIGHT_COLLECTOR_MOTOR = 13;

            // ARM
            public static final int ARM_MOTOR = 10;

            // WRIST
            public static final int WRIST_MOTOR = 11;

            // ELEVATOR
            public static final int ELEVATOR_MOTOR = 9;

            // COLECTOR
            public static final int COLLECTOR_MOTOR_ONE = 12;
            public static final int COLLECTOR_MOTOR_TWO = 13;

            // MISC SENSORS
            public static final int TIME_OF_FLIGHT = 0;
        }

        public static final class PWM {
            public static final int SERVO = 3;
        }
    }

    public static final class AutoBalanceConstants {
        public static final double MAGNITUDE_SCALER = 0.09;
        public static final double BALANCED_MAGNITUDE = 2.5;
        public static final double UPPER_MAGNITUDE_THRESHOLD = 11;
        public static final double LOWER_MAGNITUDE_THRESHOLD = 3;
        public static final double MAGNITUDE_RATE_OF_CHANGE_THRESHOLD = 0.05;
        public static final double MIN_SPEED_THRESHOLD = 0.35;
        public static final double MAX_SPEED_THRESHOLD = 3;
        public static final double DELAY_TIME = 2;

        public static final double TARGET_X = 3.93;
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class LedConstants {
        public static final int ledPort = 22;
        public static final int ledLength = 170;
        public static final double ledSpeed = .5;

        public static final class Colors {
            // lightning colors
            public static final int[] lightningOrange = { 255, 71, 15 };
            public static final int[] lightningBlue = { 0, 0, 255 };

            // misc colors
            public static final int[] cyan = { 96, 209, 149 };
            public static final int[] yellow = { 255, 230, 20 };
            public static final int[] purple = { 220, 30, 240 };
            public static final int[] green = { 0, 255, 0 };
            public static final int[] red = { 255, 0, 0 };
            public static final int[] white = { 255, 255, 255 };
            public static final int[] off = { 0, 0, 0 };
        }
    }

    public static final class VisionConstants {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in angles)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

        public static final Matrix<N3, N1> STANDARD_DEV_VISION_MATRIX = VecBuilder.fill(1.195384707229739, 0.7850610924749237, 2.2025094640913276);
    }

    public static final class LiftConstants {
        public enum LiftState {
            ground(new Translation2d(0d, 0d)), doubleSubstationCollect(
                    new Translation2d(0d, 0d)),
            reverseSubstationCollect(new Translation2d(0d, 0d)),

            mediumCubeScore(new Translation2d(0d, 0d)), highCubeScore(
                    new Translation2d(0d, 0d)),
            mediumConeScore(
                    new Translation2d(0d, 0d)),
            highConeScore(new Translation2d(0d, 0d)),

            elevatorDeployed(new Translation2d(0d, 0d)), armDeployed(new Translation2d(0d, 0d)),

            stowed(new Translation2d(0d, 0d));

            private Translation2d pose;

            LiftState(Translation2d pose) {
                this.pose = pose;
            }

            public Translation2d pose() {
                return pose;
            }
        }

        // TODO: replace with actual bounding box values
        public static final Polygon BOUNDING_BOX = new Polygon(new int[] { 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0 }, 4);
    }

    public static final class ManualLiftConstants {
        public static final double ELEVATOR_SPEED_REDUCTION = 1;
        public static final double ARM_SPEED_REDUCTION = 0.01;
        public static final double WRIST_SPEED_REDUCTION = 0.01;
    }

    public static final class AutonomousConstants {
        // Path planner PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(10.5, 0, 0);
        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(7, 0, 0);
        public static final PIDConstants POSE_PID_CONSTANTS = new PIDConstants(0,0, 0);

        public static final double MAX_VELOCITY = 3;
        public static final double MAX_ACCELERATION = 3;
    }
}
