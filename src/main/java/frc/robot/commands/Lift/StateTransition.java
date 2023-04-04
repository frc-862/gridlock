package frc.robot.commands.Lift;

import org.apache.commons.lang3.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftPlan;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.Constants.WristConstants.WRIST_SCHEDULE;
import frc.robot.Constants.WristConstants;

/**
 * A class that represents a state transition for the lift
 */
public class StateTransition {
    private double elevatorExtension;
    private Rotation2d armAngle;
    private Rotation2d wristAngle;
    private LiftPlan plan;
    private LiftState endState;
    private Range<Double> elevatorSafeZone;
    private Range<Double> armSafeZone;
    private Range<Double> wristSafeZone;
    private WRIST_SCHEDULE wristSchedule;



    /**
     * Creates a new state transition
     * 
     * @param elevatorExtension The extension of the elevator
     * @param armAngle The angle of the arm
     * @param wristAngle The angle of the wrist
     * @param plan The plan for the lift
     * @param endState The end state of the lift
     */
    public StateTransition(double elevatorExtension, Rotation2d armAngle, Rotation2d wristAngle, LiftPlan plan, LiftState endState, Range<Double> elevatorSafeZone, Range<Double> armSafeZone,
            Range<Double> wristSafeZone, WRIST_SCHEDULE wristSchedule) {
        this.elevatorExtension = elevatorExtension;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.plan = plan;
        this.endState = endState;
        this.elevatorSafeZone = elevatorSafeZone;
        this.armSafeZone = armSafeZone;
        this.wristSafeZone = wristSafeZone;
        this.wristSchedule = wristSchedule;
    }

    /**
     * Creates a new state transition
     * 
     * @param elevatorExtension The extension of the elevator
     * @param armAngle The angle of the arm
     * @param wristAngle The angle of the wrist
     * @param plan The plan for the lift
     * @param endState The end state of the lift
     */
    public StateTransition(double elevatorExtension, Rotation2d armAngle, Rotation2d wristAngle, LiftPlan plan, LiftState endState) {
        this(elevatorExtension, armAngle, wristAngle, plan, endState, Range.between(ElevatorConstants.MIN_EXTENSION, ElevatorConstants.MAX_EXTENSION),
                Range.between(ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE), Range.between(WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE), WRIST_SCHEDULE.SMALL_MOVEMENT);
    }

    public StateTransition(double elevatorExtension, Rotation2d armAngle, Rotation2d wristAngle, LiftPlan plan, LiftState endState, Range<Double> elevatorSafeZone, Range<Double> armSafeZone,
    Range<Double> wristSafeZone) {
        this(elevatorExtension, armAngle, wristAngle, plan, endState,
        elevatorSafeZone, armSafeZone, wristSafeZone, WRIST_SCHEDULE.SMALL_MOVEMENT);
    }

    public StateTransition(double elevatorExtension, Rotation2d armAngle, Rotation2d wristAngle, LiftPlan plan, LiftState endState, WRIST_SCHEDULE wristSchedule) {
        this(elevatorExtension, armAngle, wristAngle, plan, endState, Range.between(ElevatorConstants.MIN_EXTENSION, ElevatorConstants.MAX_EXTENSION),
                Range.between(ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE), Range.between(WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE), wristSchedule);
    }

    /**
     * Gets the extension of the elevator
     * 
     * @return The extension of the elevator
     */
    public double getElevatorExtension() {
        return elevatorExtension;
    }

    /**
     * Gets the angle of the arm
     * 
     * @return The angle of the arm
     */
    public Rotation2d getArmAngle() {
        return armAngle;
    }

    /**
     * Gets the angle of the wrist
     * 
     * @return The angle of the wrist
     */
    public Rotation2d getWristAngle() {
        return wristAngle;
    }

    /**
     * Gets the plan for the lift
     * 
     * @return The plan for the lift
     */
    public LiftPlan getPlan() {
        return plan;
    }

    /**
     * Gets the end state of the lift
     * 
     * @return The end state of the lift
     */
    public LiftState getEndState() {
        return endState;
    }

    /**
     * Checks if the input is in the safe zone
     * 
     * @param input The input to check
     * @return True if the input is in the safe zone, false otherwise
     */
    public boolean isInWristSafeZone(double input) {
        return wristSafeZone.contains(input);
    }

    /**
     * Checks if the input is in the safe zone
     * 
     * @param input The input to check
     * @return True if the input is in the safe zone, false otherwise
     */
    public boolean isInArmSafeZone(double input) {
        return armSafeZone.contains(input);
    }

    /**
     * Checks if the input is in the safe zone
     * 
     * @param input The input to check
     * @return True if the input is in the safe zone, false otherwise
     */
    public boolean isInEleSafeZone(double input) {
        return elevatorSafeZone.contains(input);
    }

    public WRIST_SCHEDULE getWristSchedule() {
        return wristSchedule;
    }
}
