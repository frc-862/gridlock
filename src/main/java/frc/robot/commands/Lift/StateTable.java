package frc.robot.commands.Lift;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.LiftPlan;
import frc.robot.Constants.LiftConstants.LiftState;

/**
 * A table of all possible state transitions
 */
public class StateTable {
    // A map of all possible state transitions
    private static Map<LiftState, StateTransition> nextStateTable = Map.of(
            LiftState.stowed, new StateTransition(LiftConstants.ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(LiftConstants.ARM_STOWED_ANGLE), Rotation2d.fromDegrees(LiftConstants.WRIST_STOWED_ANGLE),
                    LiftPlan.elevatorLast, LiftState.stowed),
            LiftState.transition, new StateTransition(LiftConstants.ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(LiftConstants.ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(LiftConstants.WRIST_TRANSITION_ANGLE),
                    LiftPlan.parallel, LiftState.transition),
            LiftState.ground, new StateTransition(0, Rotation2d.fromDegrees(-70.5), Rotation2d.fromDegrees(30.0), LiftPlan.wristPriority, LiftState.ground), 
            LiftState.midCubeScore, new StateTransition(8, Rotation2d.fromDegrees(-15), Rotation2d.fromDegrees(-40), LiftPlan.parallel, LiftState.midCubeScore), 
            LiftState.midConeScore, new StateTransition(0, Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(-75), LiftPlan.parallel, LiftState.midConeScore), 
            LiftState.highCubeScore, new StateTransition(22, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-70), LiftPlan.parallel, LiftState.highCubeScore), 
            LiftState.highConeScore, new StateTransition(24, Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(-75), LiftPlan.parallel, LiftState.highConeScore), 
            LiftState.doubleSubstationCollect, new StateTransition(5, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), LiftPlan.parallel, LiftState.doubleSubstationCollect), 
            LiftState.reverseSubstationCollect, new StateTransition(5, Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(0), LiftPlan.parallel, LiftState.reverseSubstationCollect));

    // Index is current state, goal state; outputs a StateTransition
    private static Map<LiftState, Map<LiftState, StateTransition>> stateTable =
            Map.of(LiftState.transition, nextStateTable, LiftState.ground, nextStateTable, LiftState.midCubeScore, nextStateTable, LiftState.midConeScore, nextStateTable, LiftState.highCubeScore,
                    nextStateTable, LiftState.highConeScore, nextStateTable, LiftState.doubleSubstationCollect, nextStateTable, LiftState.reverseSubstationCollect, nextStateTable);

    /**
     * Gets the state transition from the current state to the goal state
     * 
     * @param current The current state
     * @param goal The goal state
     * 
     * @return The state transition
     */
    public static StateTransition get(LiftState current, LiftState goal) {
        // If we're stowed, always go to transition
        if (current == LiftState.stowed || (goal == LiftState.stowed && current != LiftState.transition)) {
            return new StateTransition(LiftConstants.ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(LiftConstants.ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(LiftConstants.WRIST_TRANSITION_ANGLE),
                    LiftPlan.elevatorPriority, LiftState.transition);
        } else {
            return stateTable.get(current).get(goal);
        }
    }
}
