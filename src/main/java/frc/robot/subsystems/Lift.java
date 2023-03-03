package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Lift.StateTable;
import frc.robot.commands.Lift.StateTransition;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The lift subsystem
 */
public class Lift extends SubsystemBase {

    // The Elevator, Wrist, and Arm subsystems
    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    // The current and goal states
    private LiftState currentState = LiftState.stowed;
    private LiftState goalState = LiftState.stowed;

    // The next state to transition to
    private StateTransition nextState;

    private boolean doTargetOverride = false;

    // Periodic Shuffleboard 
    private LightningShuffleboardPeriodic periodicShuffleboard;
    // private LightningShuffleboardPeriodic periodicShuffleboardNextState;

    // Periodic Shuffleboard 
    private LightningShuffleboardPeriodic periodicShuffleboard;
    private LightningShuffleboardPeriodic periodicShuffleboardNextState;

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

        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Lift", LiftConstants.LOG_PERIOD, new Pair<String, Object>("Lift current state", (Supplier<String>) () -> currentState.toString()),
                new Pair<String, Object>("Lift goal state", (Supplier<String>) () -> goalState.toString()), new Pair<String, Object>("Lift on target", (BooleanSupplier) () -> onTarget()));
        if (nextState != null) {
            periodicShuffleboardNextState =
                    new LightningShuffleboardPeriodic("Lift", LiftConstants.LOG_PERIOD, new Pair<String, Object>("Lift next state elevator extension", (DoubleSupplier) () -> nextState.getElevatorExtension()),
                            new Pair<String, Object>("Lift next state arm angle", (DoubleSupplier) () -> nextState.getArmAngle().getDegrees()),
                            new Pair<String, Object>("Lift next state wrist angle", (DoubleSupplier) () -> nextState.getWristAngle().getDegrees()),
                            new Pair<String, Object>("Lift next state plan", (Supplier<String>) () -> nextState.getPlan().toString()));
        }

        // mech_elevator.setLength(elevator.getExtension());
        // mech_arm.setAngle(arm.getAngle());
        // mech_wrist.setAngle(wrist.getAngle());
        // LightningShuffleboard.set("Lift", "mechanism 2D", armMech);
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
        } else if (doTargetOverride) {
            doTargetOverride = false;
            return true;
        } else {
            return elevator.onTarget(nextState.getElevatorExtension()) && arm.onTarget(nextState.getArmAngle().getDegrees()) && wrist.onTarget(nextState.getWristAngle().getDegrees());
        }
    }

    public void targetOverride() {
        goalState = currentState;

        doTargetOverride = true;

    }

    public boolean goalReached() {
        return currentState == goalState;
    }

    public Rotation2d getWristTarg() {
        if (nextState != null) {
            return nextState.getWristAngle();
        } else {
            return Rotation2d.fromDegrees(30);
        }
    }

    public void runPeriodicShuffleboardLoop() {
        periodicShuffleboard.loop();
    }

    public void adjustArm(double angle) {
        goalState = currentState;
        nextState = null;
        arm.setAngle(arm.getAngle().plus(Rotation2d.fromDegrees(angle)));
    }

    public void adjustWrist(double angle) {
        goalState = currentState;
        nextState = null;
        wrist.setAngle(wrist.getAngle().plus(Rotation2d.fromDegrees(angle)));
    }

    public void adjustElevator(double extension) {
        goalState = currentState;
        nextState = null;
        elevator.setExtension(elevator.getExtension() + extension);
    }

    @Override
    public void periodic() {

        // Updates the shuffleboard values
        runPeriodicShuffleboardLoop();

        // Checks if were on target or if the next state is null
        // Checks if were on target or if the next state is null, also makes sure our biassese havent changed
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
                case wristPriority:
                    wrist.setAngle(nextState.getWristAngle());
                    if (wrist.onTarget()) {
                        elevator.setExtension(nextState.getElevatorExtension());
                        arm.setAngle(nextState.getArmAngle());
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

        runPeriodicShuffleboardLoop();
    }
}
