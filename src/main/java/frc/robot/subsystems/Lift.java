package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftState;

public class Lift extends SubsystemBase {

    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    public LiftState lastState = LiftState.stowed;
    public LiftState currentState = LiftState.stowed;
    public LiftState nextState = LiftState.stowed;

    private Translation2d position = new Translation2d();

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.arm = arm;

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setNextState(LiftState state) {
        this.nextState = state;
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getHeight(), ElevatorConstants.ANGLE);
    }

    public Translation2d getBarXY() {
        return new Translation2d(ArmConstants.LENGTH, new Rotation2d(arm.getAngle().getRadians()));
    }

    public Translation2d getOverallXY() {
        return ElevatorConstants.POSE_OFFSET.plus(getElevatorXY())
                .plus(getBarXY().plus(WristConstants.POSE_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    public double[] elevatorMath(Translation2d desiredPose) {

        double angle = 0;

        double xPose;
        double yPose;

        double desiredXPose = desiredPose.getX();
        double desiredYPose = desiredPose.getY();

        // Find quadratic formula values
        double aQuadraticValue = 1 + Math.pow(Math.tan(ArmConstants.ELEVATOR_ANGLE), 2);
        double bQuadraticValue =
                -2 * (desiredXPose + desiredYPose * Math.tan(ArmConstants.ELEVATOR_ANGLE));
        double cQuadraticValue = Math.pow(desiredXPose, 2) + Math.pow(desiredYPose, 2)
                - Math.pow(ArmConstants.LENGTH, 2);


        // Find possible x and y poses using quadratic formula
        double possibleXPose1 = (-bQuadraticValue + Math
                .sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue);
        double possibleXPose2 = (-bQuadraticValue - Math
                .sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue);;
        double possibleYPose1 = Math.tan(ArmConstants.ELEVATOR_ANGLE) * possibleXPose1;
        double possibleYPose2 = Math.tan(ArmConstants.ELEVATOR_ANGLE) * possibleXPose2;

        // Find the x and y poses that are within the bounds of the robot or find the closer one

        if (desiredYPose < 0) {
            double slope1 = (possibleYPose1 - desiredYPose) / (possibleXPose1 - desiredXPose);
            double slope2 = (possibleYPose2 - desiredYPose) / (possibleXPose2 - desiredXPose);
            double robotIntersectionX1 = -(desiredYPose / slope1) + desiredXPose;
            double robotIntersectionX2 = -(desiredYPose / slope2) + desiredXPose;
            if (robotIntersectionX1 < ArmConstants.ROBOT_BODY_LENGTH) {
                xPose = possibleXPose2;
                yPose = possibleYPose2;
            } else if (robotIntersectionX2 < ArmConstants.ROBOT_BODY_LENGTH) {
                xPose = possibleXPose1;
                yPose = possibleYPose1;
            } else {
                double elevatorHeight = elevator.getHeight();
                double elevatorX = elevatorHeight * Math.cos(ArmConstants.ELEVATOR_ANGLE);
                double elevatorY = elevatorHeight * Math.sin(ArmConstants.ELEVATOR_ANGLE);
                double dist1 = Math.sqrt(Math.pow(elevatorX - possibleXPose1, 2)
                        + Math.pow(elevatorY - possibleYPose1, 2));
                double dist2 = Math.sqrt(Math.pow(elevatorX - possibleXPose2, 2)
                        + Math.pow(elevatorY - possibleYPose2, 2));

                if (dist1 < dist2) {
                    xPose = possibleXPose1;
                    yPose = possibleYPose1;
                } else {
                    xPose = possibleXPose2;
                    yPose = possibleYPose2;

                }

            }

        } else {

            if (possibleXPose1 < ArmConstants.MIN_X || possibleXPose1 > ArmConstants.MAX_X
                    || possibleYPose1 < ArmConstants.MIN_X) {
                xPose = possibleXPose2;
                yPose = possibleYPose2;
            } else if (possibleXPose2 < ArmConstants.MIN_X || possibleXPose2 > ArmConstants.MAX_X
                    || possibleYPose2 < ArmConstants.MIN_X) {
                xPose = possibleXPose1;
                yPose = possibleYPose1;
            } else {
                double elevatorHeight = elevator.getHeight();
                double elevatorX = elevatorHeight * Math.cos(ArmConstants.ELEVATOR_ANGLE);
                double elevatorY = elevatorHeight * Math.sin(ArmConstants.ELEVATOR_ANGLE);
                double dist1 = Math.sqrt(Math.pow(elevatorX - possibleXPose1, 2)
                        + Math.pow(elevatorY - possibleYPose1, 2));
                double dist2 = Math.sqrt(Math.pow(elevatorX - possibleXPose2, 2)
                        + Math.pow(elevatorY - possibleYPose2, 2));

                if (dist1 < dist2) {
                    xPose = possibleXPose1;
                    yPose = possibleYPose1;
                } else {
                    xPose = possibleXPose2;
                    yPose = possibleYPose2;

                }

            }
        }

        // Find the angle of the arm pivot
        if (desiredYPose == yPose) {

        } else if (desiredYPose > yPose) {
            angle = 180 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
            angle += Math.toDegrees(Math.atan((desiredYPose - yPose) / (desiredXPose - xPose)));
        } else if (desiredXPose > xPose) {
            angle = 90 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
            angle += Math.toDegrees(Math.atan((desiredXPose - xPose) / (yPose - desiredYPose)));

        } else {
            angle = 90 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
            angle -= Math.toDegrees(Math.atan((desiredXPose - xPose) / (desiredYPose - yPose)));
        }
        double elevatorLength = Math.sqrt(Math.pow(xPose, 2) + Math.pow(yPose, 2));

        double[] returnValue = {MathUtil.clamp(angle,ArmConstants.MIN_ANGLE,ArmConstants.MAX_ANGLE), elevatorLength};
        return returnValue;
    }

    public boolean isFinished() {
        return getElevatorXY() == currentState.pose(); // TODO: add some kind of tolerance
    }

    @Override
    public void periodic() {
        if (lastState != nextState && lastState == LiftState.stowed
                || currentState == LiftState.elevatorDeployed) {
            currentState = LiftState.elevatorDeployed;

            if (isFinished()) {
                currentState = nextState;
            }

        } else {
            currentState = nextState;
        }


        switch (currentState) {
            // collect states
            case ground:
                position = LiftState.ground.pose();
                break;

            case doubleSubstationCollect:
                position = LiftState.doubleSubstationCollect.pose();
                break;

            case reverseSubstationCollect:
                position = LiftState.reverseSubstationCollect.pose();
                break;


            // scoring states
            case mediumCubeScore:
                position = LiftState.mediumCubeScore.pose();
                break;

            case highCubeScore:
                position = LiftState.highCubeScore.pose();
                break;

            case mediumConeScore:
                position = LiftState.mediumConeScore.pose();
                break;

            case highConeScore:
                position = LiftState.highConeScore.pose();
                break;


            // substates
            case elevatorDeployed:
                position = LiftState.elevatorDeployed.pose();
                break;

            case armDeployed:
                position = LiftState.armDeployed.pose();
                break;



            case stowed:
                position = LiftState.stowed.pose();
                break;
        }

        if (isReachable(position)) {

            double[] liftInfo = elevatorMath(position);

            elevator.setHeight(liftInfo[1]);
            arm.setAngle(new Rotation2d(liftInfo[0]));
            wrist.setAngle(new Rotation2d(liftInfo[0] + 90)); // math
        }
    }
}


