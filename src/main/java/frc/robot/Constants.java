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

    public static final class DrivetrainConstants {

        // TODO set the track width and wheel base

        // Our drivetrain and track width
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(0d);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(0d);

        // Drivetrain PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS =
                new PIDConstants(Gains.kP, Gains.kI, Gains.kD);
        public static final PIDConstants THETA_PID_CONSTANTS =
                new PIDConstants(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

        // Stopped module constants
        public static final double FRONT_LEFT_RESTING_ANGLE = -45d;
        public static final double FRONT_RIGHT_RESTING_ANGLE = 45d;
        public static final double BACK_LEFT_RESTING_ANGLE = 45d;
        public static final double BACK_RIGHT_RESTING_ANGLE = -45d;

        // Our max voltage, velocity, and angular velocity
        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
                6380.0 / 60.0 * SdsModuleConfigurations.MK4_L3.getDriveReduction()
                        * SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        // Module configuration constants
        public static final double DRIVE_CURRENT_LIMIT = 0d;
        public static final double STEER_CURRENT_LIMIT = 0d;
        public static final double NOMINAL_VOLTAGE = 0d;


        // TODO add all the submodule IDs

        // Front left moudle configurations
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toDegrees(0d);

        // Front right moudle configurations
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toDegrees(0d);

        // Back left moudle configurations
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toDegrees(0d);

        // Back right moudle configurations
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toDegrees(0d);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.15;
            public static final double kI = 0d;
            public static final double kD = 0d;

            // TODO: get these values in after characterization
            public static final double kS = 0d;
            public static final double kV = 0d;
            public static final double kA = 0d;
        }

        // Gains vaules for ProfiledPIDControllers
        public static final class ThetaGains {
            public static final double kP = 4d;
            public static final double kI = 0d;
            public static final double kD = 0.05;

        }
    }

    public static final class RobotMap {
        public static final int PIGEON_ID = 0;
    }
}
