package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.thunder.swervelib.SdsModuleConfigurations;
import java.awt.Polygon;

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

        public static final class ElevatorConstants {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final int TICKS = 42;
            public static final double GEAR_RATIO = 1d/1d;
            public static final double INCHES_PER_REV = 1d;

            public static final double MAX_HEIGHT = 0d; 
            public static final double MIN_HEIGHT = 100d;
        }

        public static final class ArmConstants {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;


            public static final int TICKS = 42;
            public static final double GEAR_RATIO = 1.0/1.0;

            public static final double MAX_ANGLE = 90d; 
            public static final double MIN_ANGLE = -90d;
        }

        public static final class WristConstants {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;


            public static final int TICKS = 42;
            public static final double GEAR_RATIO = 1.0/1.0;

            public static final double MAX_ANGLE = 90d; 
            public static final double MIN_ANGLE = -90d;
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

        public static final class Offsets {
            public static final class Gridlock {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(342.246);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(302.959);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(131.660);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(12.744);
            }

            public static final class Blackout {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(84.832);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(192.7441);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(19.5996);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(63.457);
            }
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

            //COLLECTOR
            public static final int LEFT_COLLECTOR_MOTOR = 0;
            public static final int RIGHT_COLLECTOR_MOTOR = 0;

            //ARM
            public static final int ARM_MOTOR = 0;

            //WRIST
            public static final int WRIST_MOTOR = 0;
            
            //ELEVATOR
            public static final int ELEVATOR_MOTOR = 0;
        }
    }

    public static final class LedConstants {
        public static final int port = 9;
        public static final int length = 162;
        public static final double brightness = 0.75;

        public static final class Colors {
            //lightning colors
            public static final int[] lightningOrange = {255, 71, 15};
            public static final int[] lightningBlue = {0, 0, 255};

            //misc colors
            public static final int[] cyan = {96, 209, 149};
            public static final int[] yellow = {255, 230, 20};
            public static final int[] purple = {220, 30, 240};
            public static final int[] green = {0, 255, 0};
            public static final int[] red = {255, 0, 0};
            public static final int[] white = {255, 255, 255};
            public static final int[] off = {0, 0, 0};
        }
    }
    
    public static final class Vision {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in angles)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

    }
    
    public static final class LiftStates{
        public static final Translation2d GROUND_COLLECT = new Translation2d(0d, 0d);
        public static final Translation2d DOUBLE_SUBSTATION_COLLECT = new Translation2d(0d, 0d);
        public static final Translation2d LOW_SCORE = new Translation2d(0d, 0d);
        public static final Translation2d MEDIUM_SCORE = new Translation2d(0d, 0d);
        public static final Translation2d HIGH_SCORE = new Translation2d(0d, 0d);
        public static final Translation2d STOWED = new Translation2d(0d, 0d);
    }

    public static final class XYConstants {
        public static final double ARM_RADIUS = 0; //arm length in inches
        public static final Rotation2d ELEVATOR_ANGLE = new Rotation2d(0); //Acute Elevator mount angle in degrees
        public static final Translation2d ELEVATOR_OFFSET = new Translation2d(0, 0); //horiz/vert offset from ground (See below)
        //X = distance from arm pivot point to front of bot at bottom limit (negative)
        //Y = height of arm pivot point from ground at bottom limit
        public static final Translation2d COLLECTOR_OFFSET = new Translation2d(0, new Rotation2d(0));
        public static final Polygon BOUNDING_BOX = new Polygon(new int[] {0, 0, 0, 0}, new int[] {0, 0, 0, 0}, 4);
    }
}
