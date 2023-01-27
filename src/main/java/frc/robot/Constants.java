package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.util.Units;
import frc.thunder.swervelib.SdsModuleConfigurations;

public final class Constants {

    // Constants for xbox controlers
    public static final class XboxControllerConstants {
        public static final double DEADBAND = 0.15;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;

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

        // Drivetrain PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS =
                new PIDConstants(Gains.kP, Gains.kI, Gains.kD);
        public static final PIDConstants THETA_PID_CONSTANTS =
                new PIDConstants(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

        // Module resting/default angles
        public static final double FRONT_LEFT_RESTING_ANGLE = Math.toRadians(-45d);
        public static final double FRONT_RIGHT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_LEFT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_RIGHT_RESTING_ANGLE = Math.toRadians(-45d);

        // Our max voltage, velocity, angular velocity, and angular acceleration
        public static final double MAX_VOLTAGE = 12.0;
        // TODO look at the calculation here
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
                5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND =
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2 * Math.PI;

        // Module configuration constants
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int STEER_CURRENT_LIMIT = 30;
        public static final double NOMINAL_VOLTAGE = 12d;

        // Gains vaules for PIDControllers
        public static final class Gains {

            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kS = 0.13;
            public static final double kV = 2.64;
            public static final double kA = 0;
        }

        // Gains vaules for theta PIDControllers
        public static final class ThetaGains {
            public static final double kP = 0.004d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kS = 0d;
            public static final double kV = 0d;
            public static final double kA = 0d;

        }
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
        }
    }

    public static final class Vision {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in angles)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

    }
}
