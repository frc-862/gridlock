package frc.robot.subsystems;

import java.util.Arrays;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Lift.StateTable;
import frc.robot.commands.Lift.StateTransition;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Lift extends SubsystemBase {

    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    public LiftState currentState = LiftState.stowed;
    public LiftState goalState = LiftState.stowed;
    public StateTransition nextState;

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.arm = arm;

        elevator.setExtension(elevator.getExtension());
        arm.setAngle(arm.getAngle());
        wrist.setAngle(wrist.getAngle());

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {}

    public void setGoalState(LiftState state) {
        this.goalState = state;
    }

    public boolean onTarget() {
        if (nextState == null) {
            return elevator.onTarget() && arm.onTarget() && wrist.onTarget();
        } else {
            return elevator.onTarget(nextState.getElevatorExtension()) && arm.onTarget(nextState.getArmAngle().getDegrees()) && wrist.onTarget(nextState.getWristAngle().getDegrees());
        }
    }

    @Override
    public void periodic() {
        if (onTarget() || nextState == null) {
            if (currentState != goalState) {
                nextState = StateTable.get(currentState, goalState);
            } else {
                nextState = null;
            }

            if (nextState != null) {
                currentState = nextState.getEndState();
            }
        } else {
            switch (nextState.getPlan()) {
                case parallel:
                    elevator.setExtension(nextState.getElevatorExtension());
                    arm.setAngle(nextState.getArmAngle());
                    wrist.setAngle(nextState.getWristAngle());
                    break;
                case armPriority:
                    arm.setAngle(nextState.getArmAngle());
                    if (arm.onTarget()) {
                        elevator.setExtension(nextState.getElevatorExtension());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
                case elevatorPriority:
                    elevator.setExtension(nextState.getElevatorExtension());
                    if (elevator.onTarget()) {
                        arm.setAngle(nextState.getArmAngle());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
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

        // if(transitionState != null) {
        // LightningShuffleboard.setDouble("Lift", "ele target",
        // transitionState.getElevatorExtension());
        // LightningShuffleboard.setDouble("Lift", "arm target",
        // transitionState.getArmAngle().getDegrees());
        // LightningShuffleboard.setDouble("Lift", "wrist target",
        // transitionState.getWristAngle().getDegrees());
        // }

        LightningShuffleboard.setString("Lift", "current state", currentState.toString());
        LightningShuffleboard.setString("Lift", "goal state", goalState.toString());
        LightningShuffleboard.setBool("Lift", "on target", onTarget());
    }
}
