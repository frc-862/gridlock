package frc.robot.subsystems;

import java.util.Arrays;
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
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.logging.DataLogger;

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

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Elevator X", () -> getElevatorXY().getX());
        DataLogger.addDataElement("Elevator Y", () -> getElevatorXY().getY());
        DataLogger.addDataElement("Arm X", () -> getArmXY().getX());
        DataLogger.addDataElement("Arm Y", () -> getArmXY().getY());
        DataLogger.addDataElement("Overall X", () -> getOverallXY().getX());
        DataLogger.addDataElement("Overall Y", () -> getOverallXY().getY());
        DataLogger.addDataElement("lift is finished", () -> isFinished() ? 1 : 0);
        DataLogger.addDataElement("lift is reachable", () -> isReachable(getOverallXY()) ? 1 : 0);
    }

    public void setNextState(LiftState state) {
        this.nextState = state;
    }

    /**
     * getEleveatorXY
     *
     * @return Translation2d of the elevator from it's zero point
     */
    // Gets the XY position of the elevator at arm the pivot point
    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getExtension(), ElevatorConstants.ANGLE);
    }

    /**
     * getArmXY
     *
     * @return Translation2d of the arm from it's pivot point
     */
    // Gets the XY position of the arm at the wrist pivot point, with the arm pivot as the origin
    public Translation2d getArmXY() {
        return new Translation2d(ArmConstants.LENGTH, arm.getAngle());
    }

    /**
     * getOverallXY
     *
     * @return Translation2d of the collector from the origin
     */
    // Gets the overall XY position with offsets
    public Translation2d getOverallXY() {
        return ElevatorConstants.POSE_OFFSET.plus(getElevatorXY())
                .plus(getArmXY().plus(WristConstants.POSE_OFFSET));
    }

    public double getExtensionFromPoint(Translation2d point) {
        return Math.sqrt(Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2));
    }

    public double getExtensionFromPoint(double x, double y) {
        return this.getExtensionFromPoint(new Translation2d(x, y));
    }

    /**
     * isReachable
     *
     * @param pose a desired point to check
     * @return whether the desired point is possible for the elevator to reach
     */
    // Checks if our set position is within the bounds of the robot
    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    public class liftSolution {
        public Translation2d elevatorPose;
        public Translation2d desiredPose;

        public liftSolution(double elevatorX, Translation2d desiredPose) {
            this.elevatorPose = new Translation2d(elevatorX, ElevatorConstants.ANGLE.getTan());
            this.desiredPose = desiredPose;
        }

        public double getExtension() {
            return getExtensionFromPoint(elevatorPose);
        }

        public Rotation2d getArmAngle() {
            return elevatorPose.minus(desiredPose).getAngle();
        }

        public boolean isValid() {
            return elevator.isReachable(getExtension()) && arm.isReachable(getArmAngle()) && isReachable(desiredPose);
        }
    }

    public liftSolution liftMath(Translation2d desiredPose) {
        // Find quadratic formula values
        double aQuadraticValue = 1 + Math.pow(ElevatorConstants.ANGLE.getTan(), 2);
        double bQuadraticValue = -2 * (desiredPose.getX() + desiredPose.getY() * ElevatorConstants.ANGLE.getTan());
        double cQuadraticValue = Math.pow(desiredPose.getX(), 2) + Math.pow(desiredPose.getY(), 2)
                - Math.pow(ArmConstants.LENGTH, 2);

        // Find possible x and y poses using quadratic formula (using a list because of the +/- in the square root)
        liftSolution[] possibleSolutions = {
                new liftSolution((-bQuadraticValue + 
                Math.sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue), desiredPose), 
                new liftSolution(((-bQuadraticValue - 
                Math.sqrt(bQuadraticValue * bQuadraticValue - 4 * aQuadraticValue * cQuadraticValue))
                / (2 * aQuadraticValue)), desiredPose)
        };

        //remove all invalid solutions from list
        possibleSolutions = Arrays.stream(possibleSolutions).filter(liftSolution::isValid).toArray(liftSolution[]::new);

        // if we still have 2 solutions, use the one with the lowest elevator extension
        if (possibleSolutions.length == 2) {
            if (possibleSolutions[0].getExtension() < possibleSolutions[1].getExtension()) {
                possibleSolutions = new liftSolution[] { possibleSolutions[0] };
            } else {
                possibleSolutions = new liftSolution[] { possibleSolutions[1] };
            }
        } else if (possibleSolutions.length == 0) {
            return null; //we messed up
        }
        return possibleSolutions[0];
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
    }
}
