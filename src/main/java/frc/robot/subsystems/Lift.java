package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Lift.StateTable;
import frc.robot.commands.Lift.StateTransition;
import frc.thunder.shuffleboard.LightningShuffleboard;

/**
 * The lift subsystem
 */
public class Lift extends SubsystemBase {

    // The Elevator, Wrist, and Arm subsystems
    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    // The current and goal states
    public LiftState currentState = LiftState.stowed;
    public LiftState goalState = LiftState.stowed;

    // The next state to transition to
    public StateTransition nextState;

    /**
     * Creates a new lift subsystem
     * 
     * @param elevator the elevator subsystem
     * @param wrist the wrist subsystem
     * @param arm the arm subsystem
     */
    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.arm = arm;

        // Sets the initial state of the elevator, arm, and wrist
        elevator.setExtension(elevator.getExtension());
        arm.setAngle(arm.getAngle());
        wrist.setAngle(wrist.getAngle());

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    private void updateShuffleboard() {
        LightningShuffleboard.setString("Lift", "Lift current state", currentState.toString());
        LightningShuffleboard.setString("Lift", "Lift goal state", goalState.toString());
        LightningShuffleboard.setBool("Lift", "Lift on target", onTarget());

        if(nextState != null) {
            LightningShuffleboard.setDouble("Lift", "Lift next state elevator extension", nextState.getElevatorExtension());
            LightningShuffleboard.setDouble("Lift", "Lift next state arm angle", nextState.getArmAngle().getDegrees());
            LightningShuffleboard.setDouble("Lift", "Lift next state wrist angle", nextState.getWristAngle().getDegrees());
            LightningShuffleboard.setString("Lift", "Lift next state plan", nextState.getPlan().toString());
            LightningShuffleboard.set("Lift", "Lift next state", nextState);
        }
    }

    /**
     * Sets the goal state of the lift
     * 
     * @param state the goal state
     */
    public void setGoalState(LiftState state) {
        this.goalState = state;
    }

    /**
     * Checks if the all the components of lift are on target
     * 
     * @return true if all the components of lift are on target
     */
    public boolean onTarget() {
        if (nextState == null) {
            return elevator.onTarget() && arm.onTarget() && wrist.onTarget();
        } else {
            return elevator.onTarget(nextState.getElevatorExtension()) && arm.onTarget(nextState.getArmAngle().getDegrees()) && wrist.onTarget(nextState.getWristAngle().getDegrees());
        }
    }

    public boolean goalReached() {
        return currentState == goalState;
    }

    @Override
    public void periodic() {
        // Checks if were on target or if the next state is null
        if (onTarget() || nextState == null) {
            // Checks if the current state is not the goal state
            if (currentState != goalState) {
                // Gets the next state from the state table
                nextState = StateTable.get(currentState, goalState);
            } else {
                // sets the next state to null
                nextState = null;
            }

            // Checks if the next state is not null
            if (nextState != null) {
                // Sets the current state to the end state of the next state
                currentState = nextState.getEndState();
            }
        } else {

            // Checks the run plan of the next state
            switch (nextState.getPlan()) {
                // If parallel, set all the components to their target
                case parallel:
                    elevator.setExtension(nextState.getElevatorExtension());
                    arm.setAngle(nextState.getArmAngle());
                    wrist.setAngle(nextState.getWristAngle());
                    break;
                // If armPriority, set the arm to its target and then set the elevator and wrist
                case armPriority:
                    arm.setAngle(nextState.getArmAngle());
                    if (arm.onTarget()) {
                        elevator.setExtension(nextState.getElevatorExtension());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
                // If elevatorPriority, set the elevator to its target and then set the arm and wrist
                case elevatorPriority:
                    elevator.setExtension(nextState.getElevatorExtension());
                    if (elevator.onTarget()) {
                        arm.setAngle(nextState.getArmAngle());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
                // If elevatorLast set the wrist to its target and then set the arm and lastly the elevator
                case elevatorLast:
                    wrist.setAngle(nextState.getWristAngle());
                    if (wrist.onTarget()) {
                        arm.setAngle(nextState.getArmAngle());
                        if (arm.onTarget()) {
                            elevator.setExtension(nextState.getElevatorExtension());
                        }
                    }
                    break;
            }
        }

        // Starts logging and updates the shuffleboard
        // updateShuffleboard();
    }
}
