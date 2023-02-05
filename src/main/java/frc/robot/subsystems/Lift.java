package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.DrivetrainConstants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.DrivetrainConstants.WristConstants;

public class Lift extends SubsystemBase {

    public enum States {
        groundCollect, doubleSubstationCollect, lowScore, mediumScore, highScore, stowed
    }

    Elevator elevator;
    Wrist wrist;
    Arm arm;

    public States state = States.stowed;
    private Translation2d position = new Translation2d();

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.arm = arm;
    }

    public void setState(States state) {
        this.state = state;
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getHeight(), ElevatorConstants.ANGLE);
    }

    public Translation2d getBarXY() {
        return new Translation2d(ArmConstants.LENGTH,
                new Rotation2d(Math.toRadians(arm.getAngle())));
    }

    public Translation2d getOverallXY() {
        return ElevatorConstants.OFFSET.plus(getElevatorXY())
                .plus(getBarXY().plus(WristConstants.COLLECTOR_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    public double[] elevatorMath(Translation2d desiredPose) {

        double angle;

        double xPose;
        double yPose;

        double desiredXPose = desiredPose.getX();
        double desiredYPose = desiredPose.getY();

        double aQuadraticValue = 1 + Math.pow(Math.tan(ArmConstants.ELEVATOR_ANGLE), 2);
        double bQuadraticValue =
                -2 * (desiredXPose + desiredYPose * Math.tan(ArmConstants.ELEVATOR_ANGLE));
        double cQuadraticValue = Math.pow(desiredXPose, 2) + Math.pow(desiredYPose, 2)
                - Math.pow(ArmConstants.LENGTH, 2);

        double possibleXPose1 = (-bQuadraticValue + Math
                .sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue);
        double possibleXPose2 = (-bQuadraticValue - Math
                .sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue);;
        double possibleYPose1 = Math.tan(ArmConstants.ELEVATOR_ANGLE) * possibleXPose1;
        double possibleYPose2 = Math.tan(ArmConstants.ELEVATOR_ANGLE) * possibleXPose2;

        if (possibleXPose1 < 0 || possibleXPose1 > ArmConstants.MAX_X) {
            xPose = possibleXPose2;
            yPose = possibleYPose2;
        } else if (possibleXPose2 < 0 || possibleXPose2 > ArmConstants.MAX_X) {
            xPose = possibleXPose1;
            yPose = possibleYPose1;
        } else {
            xPose = Math.min(possibleXPose1, possibleXPose2);
            yPose = Math.min(possibleYPose1, possibleYPose2);
        }

        angle = 180 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
        if (desiredYPose == yPose) {

        } else if (desiredYPose > yPose) {
            angle += Math.toDegrees(Math.atan((desiredYPose - yPose) / (desiredXPose - xPose)));
        } else if (desiredXPose > desiredXPose) {
            angle = 90 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
            angle += Math.toDegrees(
                    Math.atan((desiredXPose - desiredXPose) / (desiredYPose - desiredXPose)));

        } else {
            angle = 90 - Math.toDegrees(ArmConstants.ELEVATOR_ANGLE);
            angle -= Math.toDegrees(
                    Math.atan((desiredXPose - desiredXPose) / (desiredYPose - desiredYPose)));
        }

        double[] returnValue = {angle, desiredYPose};
        return returnValue;
    }

    @Override
    public void periodic() {
        switch (state) {
            case groundCollect:
                position = LiftConstants.GROUND_COLLECT;
                break;

            case doubleSubstationCollect:
                position = LiftConstants.DOUBLE_SUBSTATION_COLLECT;
                break;

            case lowScore:
                position = LiftConstants.LOW_SCORE;
                break;

            case mediumScore:
                position = LiftConstants.MEDIUM_SCORE;
                break;

            case highScore:
                position = LiftConstants.HIGH_SCORE;
                break;

            case stowed:
                position = LiftConstants.STOWED;
                break;
        }

        if (isReachable(position)) {

            double[] liftInfo = elevatorMath(position);

            elevator.setHeight(liftInfo[1]);
            arm.setAngle(liftInfo[0]);
            wrist.setAngle(liftInfo[0] + 90); // math
        }
    }
}


