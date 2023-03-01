package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
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
    private LiftState currentState = LiftState.stowed;
    private LiftState goalState = LiftState.stowed;

    // The next state to transition to
    private StateTransition nextState;

    private double wristBias = 0;
    private double lastWristBias = 0;
    private double armBias = 0;
    private double lastArmBias = 0;

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
        // initializeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    private void initializeShuffleboard() {
        LightningShuffleboard.setStringSupplier("Lift", "Lift current state", () -> currentState.toString());
        LightningShuffleboard.setStringSupplier("Lift", "Lift goal state", () -> goalState.toString());
        LightningShuffleboard.setBoolSupplier("Lift", "Lift on target", () -> onTarget());
        LightningShuffleboard.setDoubleSupplier("Lift", "Lift next state elevator extension", () -> nextState.getElevatorExtension());
        LightningShuffleboard.setDoubleSupplier("Lift", "Lift next state arm angle", () -> nextState.getArmAngle().getDegrees());
        LightningShuffleboard.setDoubleSupplier("Lift", "Lift next state wrist angle", () -> nextState.getWristAngle().getDegrees());
        LightningShuffleboard.setStringSupplier("Lift", "Lift next state plan", () -> nextState.getPlan().toString());
    }

    public void addWristBias(double biasToAdd) {
        wristBias += biasToAdd / 50;
    }

    public void addArmBias(double biasToAdd) {
        armBias += biasToAdd / 50;
    }

    public void resetBias() {
        wristBias = 0;
        armBias = 0;
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
        // Checks if were on target or if the next state is null, also makes sure our biassese havent changed
        if (onTarget() || nextState == null && (wristBias == lastWristBias && armBias == lastArmBias)) {
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
                    arm.setAngle(nextState.getArmAngle().plus(Rotation2d.fromDegrees(armBias)));
                    wrist.setAngle(nextState.getWristAngle().plus(Rotation2d.fromDegrees(wristBias)));
                    break;
                // If armPriority, set the arm to its target and then set the elevator and wrist
                case armPriority:
                    arm.setAngle(nextState.getArmAngle().plus(Rotation2d.fromDegrees(armBias)));
                    if (arm.onTarget()) {
                        elevator.setExtension(nextState.getElevatorExtension());
                        wrist.setAngle(nextState.getWristAngle().plus(Rotation2d.fromDegrees(wristBias)));
                    }
                    break;
                // If elevatorPriority, set the elevator to its target and then set the arm and wrist
                case elevatorPriority:
                    elevator.setExtension(nextState.getElevatorExtension());
                    if (elevator.onTarget()) {
                        arm.setAngle(nextState.getArmAngle().plus(Rotation2d.fromDegrees(armBias)));
                        wrist.setAngle(nextState.getWristAngle().plus(Rotation2d.fromDegrees(wristBias)));
                    }
                    break;
                // If elevatorLast set the wrist to its target and then set the arm and lastly the elevator
                case elevatorLast:
                    wrist.setAngle(nextState.getWristAngle().plus(Rotation2d.fromDegrees(wristBias)));
                    if (wrist.onTarget()) {
                        arm.setAngle(nextState.getArmAngle().plus(Rotation2d.fromDegrees(armBias)));
                        if (arm.onTarget()) {
                            elevator.setExtension(nextState.getElevatorExtension());
                        }
                    }
                    break;
            }
        }

        lastArmBias = armBias;
        lastWristBias = wristBias;
    }
}
