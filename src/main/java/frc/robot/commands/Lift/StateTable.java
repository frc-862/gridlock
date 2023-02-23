package frc.robot.commands.Lift;

import java.util.Map;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LiftConstants.LiftPlan;
import frc.robot.Constants.LiftConstants.LiftState;

public class StateTable {
    /*
     * public enum LiftState { ground,
     * 
     * doubleSubstationCollect, reverseSubstationCollect,
     * 
     * midCubeScore, highCubeScore, midConeScore, highConeScore,
     * 
     * elevatorDeployed, armDeployed, transition,
     * 
     * stowed, undetermined }
     */

     private static Map<LiftState, StateTransition> nextStateTable = Map.of(
                LiftState.transition, new StateTransition(4, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-20), LiftPlan.parallel, LiftState.transition),
                LiftState.ground, new StateTransition(0, Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(-90), LiftPlan.armPriority, LiftState.ground),
                LiftState.midCubeScore, new StateTransition(10, Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-120), LiftPlan.parallel, LiftState.midCubeScore),
                LiftState.midConeScore, new StateTransition(10, Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-120), LiftPlan.parallel, LiftState.midConeScore),
                LiftState.highCubeScore, new StateTransition(20, Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-120), LiftPlan.parallel, LiftState.highCubeScore),
                LiftState.highConeScore, new StateTransition(20, Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-120), LiftPlan.parallel, LiftState.highConeScore),
                LiftState.doubleSubstationCollect, new StateTransition(5, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-130), LiftPlan.parallel, LiftState.doubleSubstationCollect),
                LiftState.reverseSubstationCollect, new StateTransition(5, Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(-130), LiftPlan.parallel, LiftState.reverseSubstationCollect));

    // index is current state, goal state; outputs a StateTransition
    private static Map<LiftState, Map<LiftState, StateTransition>> stateTable = Map.of(
            LiftState.transition, nextStateTable,
            LiftState.ground, nextStateTable,
            LiftState.midCubeScore, nextStateTable,
            LiftState.midConeScore, nextStateTable,
            LiftState.highCubeScore, nextStateTable,
            LiftState.highConeScore, nextStateTable,
            LiftState.doubleSubstationCollect, nextStateTable,
            LiftState.reverseSubstationCollect, nextStateTable
            );

            

            public static StateTransition get(LiftState current, LiftState goal) {
                //if we're stowed, always go to transition
                if(current == LiftState.stowed) {
                    return new StateTransition(4, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-20), LiftPlan.elevatorPriority, LiftState.transition);
                } else if (goal == LiftState.stowed) {
                    return new StateTransition(4, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-20), LiftPlan.elevatorLast, LiftState.stowed);
                } else {
                    return stateTable.get(current).get(goal);
                }
            }
}
