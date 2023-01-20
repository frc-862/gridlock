package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import frc.thunder.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.util.Units;

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
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24d);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24d);

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
                6380.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        // Module configuration constants
        public static final double DRIVE_CURRENT_LIMIT = 40d;
        public static final double STEER_CURRENT_LIMIT = 30d;
        public static final double NOMINAL_VOLTAGE = 12d;


        // TODO add all the submodule IDs

        // Module steer offsets
        public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(84.727);
        public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(57.920);
        public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(16.348);
        public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(66.445);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.15;
            public static final double kI = 0d;
            public static final double kD = 0d;

            // TODO: get these values in after characterization
            public static final double kS = 0.59292;
            public static final double kV = 2.7301;
            public static final double kA = 0.19945;
        }

        // Gains vaules for ProfiledPIDControllers
        public static final class ThetaGains {
            public static final double kP = 4d;
            public static final double kI = 0d;
            public static final double kD = 0.05;

        }
    }

    public static final class RobotMap {
        public static final class CAN {
            public static final int PIGEON_ID = 23;
            public static final int PDH = 21;

            // FL
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 2;
            public static final int FRONT_LEFT_CANCODER = 31;
            // FR
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 32;
            // BR
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 33;
            // BL
            public static final int BACK_LEFT_DRIVE_MOTOR = 7;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 8;
            public static final int BACK_LEFT_CANCODER = 34;
        }
    }

    // Auto Balance constants
    public static final double AB_MAX_AVERAGE_DEVIATION = 3;
    public static final double AB_KP = 0.1;
    public static final double AB_KI = 0;
    public static final double AB_KD = 0;
    
}
