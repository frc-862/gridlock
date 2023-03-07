package frc.robot.commands.Lift;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftPlan;
import frc.robot.Constants.LiftConstants.LiftState;

/**
 * A table of all possible state transitions
 */
public class StateTable {

    private static final double ELEVATOR_STOWED_POS = 4;
    private static final double ARM_STOWED_ANGLE = -117;
    private static final double WRIST_STOWED_ANGLE = 112;
    private static final LiftPlan STOWED_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_STOWED_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_STOWED_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_STOWED_TOLERANCE = WristConstants.TOLERANCE;

    private static final double ELEVATOR_TRANSITION_POS = 8;
    private static final double ARM_TRANSITION_ANGLE = -70;
    private static final double WRIST_TRANSITION_ANGLE = 112;
    private static final LiftPlan TRANSITION_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_TRANSITION_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_TRANSITION_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_TRANSITION_TOLERANCE = WristConstants.TOLERANCE;
    
    private static final double ELEVATOR_GROUND_CONE_POS = 0;
    private static final double ARM_GROUND_CONE_ANGLE = -70.5;
    private static final double WRIST_GROUND_CONE_ANGLE = 30d;
    private static final LiftPlan GROUND_CONE_PLAN = LiftPlan.wristPriority;
    private static final double ELEVATOR_GROUND_CONE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_GROUND_CONE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_GROUND_CONE_TOLERANCE = WristConstants.TOLERANCE;
    
    private static final double ELEVATOR_GROUND_CUBE_POS = 5.5;
    private static final double ARM_GROUND_CUBE_ANGLE = -70.5;
    private static final double WRIST_GROUND_CUBE_ANGLE = 0;
    private static final LiftPlan GROUND_CUBE_PLAN = LiftPlan.elevatorPriority;
    private static final double ELEVATOR_GROUND_CUBE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_GROUND_CUBE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_GROUND_CUBE_TOLERANCE = WristConstants.TOLERANCE;

    private static final double ELEVATOR_MID_CUBE_SCORE_POS = 8;
    private static final double ARM_MID_CUBE_SCORE_ANGLE = -8;
    private static final double WRIST_MID_CUBE_SCORE_ANGLE = -40d;
    private static final LiftPlan MID_CUBE_SCORE_PLAN = LiftPlan.wristPriority;
    private static final double ELEVATOR_MID_CUBE_SCORE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_MID_CUBE_SCORE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_MID_CUBE_SCORE_TOLERANCE = WristConstants.TOLERANCE;
    

    private static final double ELEVATOR_MID_CONE_POS = 0;
    private static final double ARM_MID_CONE_ANGLE = 20;
    private static final double WRIST_MID_CONE_ANGLE = -69d;
    private static final LiftPlan MID_CONE_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_MID_CONE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_MID_CONE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_MID_CONE_TOLERANCE = WristConstants.TOLERANCE;


    private static final double ELEVATOR_HIGH_CUBE_POS = 22;
    private static final double ARM_HIGH_CUBE_ANGLE = 0d;
    private static final double WRIST_HIGH_CUBE_ANGLE = -69d;
    private static final LiftPlan HIGH_CUBE_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_HIGH_CUBE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_HIGH_CUBE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_HIGH_CUBE_TOLERANCE = WristConstants.TOLERANCE;

    private static final double ELEVATOR_HIGH_CONE_POS = 24;
    private static final double ARM_HIGH_CONE_ANGLE = 10d;
    private static final double WRIST_HIGH_CONE_ANGLE = -69d;
    private static final LiftPlan HIGH_CONE_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_HIGH_CONE_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_HIGH_CONE_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_HIGH_CONE_TOLERANCE = WristConstants.TOLERANCE;

    private static final double ELEVATOR_DOUBLE_SUB_POS = 5;
    private static final double ARM_DOUBLE_SUB_ANGLE = 0d;
    private static final double WRIST_DOUBLE_SUB_ANGLE = 0d;
    private static final LiftPlan DOUBLE_SUB_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_DOUBLE_SUB_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_DOUBLE_SUB_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_DOUBLE_SUB_TOLERANCE = WristConstants.TOLERANCE;

    private static final double ELEVATOR_REV_DOUBLE_SUB_POS = 5;
    private static final double ARM_REV_DOUBLE_SUB_ANGLE = 60d;
    private static final double WRIST_REV_DOUBLE_SUB_ANGLE = 0d;
    private static final LiftPlan REV_DOUBLE_SUB_PLAN = LiftPlan.parallel;
    private static final double ELEVATOR_REV_DOUBLE_SUB_TOLERANCE = ElevatorConstants.TOLERANCE;
    private static final double ARM_REV_DOUBLE_SUB_TOLERANCE = ArmConstants.TOLERANCE;
    private static final double WRIST_REV_DOUBLE_SUB_TOLERANCE = WristConstants.TOLERANCE;


    // A map of all possible state transitions
    private static Map<LiftState, StateTransition> defaultTable = Map.of(
            LiftState.stowed, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), TRANSITION_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
            LiftState.transition, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), TRANSITION_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
            LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CUBE_ANGLE), GROUND_CUBE_PLAN, LiftState.groundCube, ELEVATOR_GROUND_CUBE_TOLERANCE, ARM_GROUND_CUBE_TOLERANCE, WRIST_GROUND_CUBE_TOLERANCE), 
            LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CONE_ANGLE), GROUND_CONE_PLAN, LiftState.groundCone, ELEVATOR_GROUND_CONE_TOLERANCE, ARM_GROUND_CONE_TOLERANCE, WRIST_GROUND_CONE_TOLERANCE), 
            LiftState.midCubeScore, new StateTransition(ELEVATOR_MID_CUBE_SCORE_POS, Rotation2d.fromDegrees(ARM_MID_CUBE_SCORE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CUBE_SCORE_ANGLE), MID_CUBE_SCORE_PLAN, LiftState.midCubeScore, ELEVATOR_MID_CUBE_SCORE_TOLERANCE, ARM_MID_CUBE_SCORE_TOLERANCE, WRIST_MID_CUBE_SCORE_TOLERANCE), 
            LiftState.midConeScore, new StateTransition(ELEVATOR_MID_CONE_POS, Rotation2d.fromDegrees(ARM_MID_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CONE_ANGLE), MID_CONE_PLAN, LiftState.midConeScore, ELEVATOR_MID_CONE_TOLERANCE, ARM_MID_CONE_TOLERANCE, WRIST_MID_CONE_TOLERANCE), 
            LiftState.highCubeScore, new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), HIGH_CUBE_PLAN, LiftState.highCubeScore, ELEVATOR_HIGH_CUBE_TOLERANCE, ARM_HIGH_CUBE_TOLERANCE, WRIST_HIGH_CUBE_TOLERANCE),
            LiftState.highConeScore, new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), HIGH_CONE_PLAN, LiftState.highConeScore, ELEVATOR_HIGH_CONE_TOLERANCE, ARM_HIGH_CONE_TOLERANCE, WRIST_HIGH_CONE_TOLERANCE), 
            LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_DOUBLE_SUB_ANGLE), DOUBLE_SUB_PLAN, LiftState.doubleSubstationCollect, ELEVATOR_DOUBLE_SUB_TOLERANCE, ARM_DOUBLE_SUB_TOLERANCE, WRIST_DOUBLE_SUB_TOLERANCE), 
            LiftState.reverseSubstationCollect, new StateTransition(ELEVATOR_REV_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_REV_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_REV_DOUBLE_SUB_ANGLE), REV_DOUBLE_SUB_PLAN, LiftState.reverseSubstationCollect, ELEVATOR_REV_DOUBLE_SUB_TOLERANCE, ARM_REV_DOUBLE_SUB_TOLERANCE, WRIST_REV_DOUBLE_SUB_TOLERANCE));

        private static Map<LiftState, StateTransition> safeCollectTable = Map.of(
            LiftState.stowed, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
            LiftState.transition, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), TRANSITION_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),

            LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(112), GROUND_CUBE_PLAN, LiftState.groundCube, ELEVATOR_GROUND_CUBE_TOLERANCE, ARM_GROUND_CUBE_TOLERANCE, WRIST_GROUND_CUBE_TOLERANCE), 

            LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(112), GROUND_CONE_PLAN, LiftState.safeCollect, ELEVATOR_GROUND_CONE_TOLERANCE, ARM_GROUND_CONE_TOLERANCE, WRIST_GROUND_CONE_TOLERANCE), 

            LiftState.midCubeScore, new StateTransition(ELEVATOR_MID_CUBE_SCORE_POS, Rotation2d.fromDegrees(ARM_MID_CUBE_SCORE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CUBE_SCORE_ANGLE), MID_CUBE_SCORE_PLAN, LiftState.midCubeScore, ELEVATOR_MID_CUBE_SCORE_TOLERANCE, ARM_MID_CUBE_SCORE_TOLERANCE, WRIST_MID_CUBE_SCORE_TOLERANCE), 
            LiftState.midConeScore, new StateTransition(ELEVATOR_MID_CONE_POS, Rotation2d.fromDegrees(ARM_MID_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CONE_ANGLE), MID_CONE_PLAN, LiftState.midConeScore, ELEVATOR_MID_CONE_TOLERANCE, ARM_MID_CONE_TOLERANCE, WRIST_MID_CONE_TOLERANCE), 
            LiftState.highCubeScore, new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), HIGH_CUBE_PLAN, LiftState.highCubeScore, ELEVATOR_HIGH_CUBE_TOLERANCE, ARM_HIGH_CUBE_TOLERANCE, WRIST_HIGH_CUBE_TOLERANCE),
            LiftState.highConeScore, new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), HIGH_CONE_PLAN, LiftState.highConeScore, ELEVATOR_HIGH_CONE_TOLERANCE, ARM_HIGH_CONE_TOLERANCE, WRIST_HIGH_CONE_TOLERANCE), 
            LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_DOUBLE_SUB_ANGLE), DOUBLE_SUB_PLAN, LiftState.doubleSubstationCollect, ELEVATOR_DOUBLE_SUB_TOLERANCE, ARM_DOUBLE_SUB_TOLERANCE, WRIST_DOUBLE_SUB_TOLERANCE), 
            LiftState.reverseSubstationCollect, new StateTransition(ELEVATOR_REV_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_REV_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_REV_DOUBLE_SUB_ANGLE), REV_DOUBLE_SUB_PLAN, LiftState.reverseSubstationCollect, ELEVATOR_REV_DOUBLE_SUB_TOLERANCE, ARM_REV_DOUBLE_SUB_TOLERANCE, WRIST_REV_DOUBLE_SUB_TOLERANCE));

    

    // Index is current state, goal state; outputs a StateTransition
    private static Map<LiftState, Map<LiftState, StateTransition>> stateTable = Map.ofEntries(
            Map.entry(LiftState.transition, Map.of(
                LiftState.stowed, new StateTransition(ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), STOWED_PLAN, LiftState.stowed, ELEVATOR_STOWED_TOLERANCE, ARM_STOWED_TOLERANCE, WRIST_STOWED_TOLERANCE),
                LiftState.transition, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), TRANSITION_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CUBE_ANGLE), GROUND_CUBE_PLAN, LiftState.groundCube, ELEVATOR_GROUND_CUBE_TOLERANCE, ARM_GROUND_CUBE_TOLERANCE, WRIST_GROUND_CUBE_TOLERANCE), 
                LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CONE_ANGLE), GROUND_CONE_PLAN, LiftState.groundCone, ELEVATOR_GROUND_CONE_TOLERANCE, ARM_GROUND_CONE_TOLERANCE, WRIST_GROUND_CONE_TOLERANCE), 
                LiftState.midCubeScore, new StateTransition(ELEVATOR_MID_CUBE_SCORE_POS, Rotation2d.fromDegrees(ARM_MID_CUBE_SCORE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CUBE_SCORE_ANGLE), MID_CUBE_SCORE_PLAN, LiftState.midCubeScore, ELEVATOR_MID_CUBE_SCORE_TOLERANCE, ARM_MID_CUBE_SCORE_TOLERANCE, WRIST_MID_CUBE_SCORE_TOLERANCE), 
                LiftState.midConeScore, new StateTransition(ELEVATOR_MID_CONE_POS, Rotation2d.fromDegrees(ARM_MID_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CONE_ANGLE), MID_CONE_PLAN, LiftState.midConeScore, ELEVATOR_MID_CONE_TOLERANCE, ARM_MID_CONE_TOLERANCE, WRIST_MID_CONE_TOLERANCE), 
                LiftState.highCubeScore, new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), HIGH_CUBE_PLAN, LiftState.highCubeScore, ELEVATOR_HIGH_CUBE_TOLERANCE, ARM_HIGH_CUBE_TOLERANCE, WRIST_HIGH_CUBE_TOLERANCE),
                LiftState.highConeScore, new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), HIGH_CONE_PLAN, LiftState.highConeScore, ELEVATOR_HIGH_CONE_TOLERANCE, ARM_HIGH_CONE_TOLERANCE, WRIST_HIGH_CONE_TOLERANCE), 
                LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_DOUBLE_SUB_ANGLE), DOUBLE_SUB_PLAN, LiftState.doubleSubstationCollect, ELEVATOR_DOUBLE_SUB_TOLERANCE, ARM_DOUBLE_SUB_TOLERANCE, WRIST_DOUBLE_SUB_TOLERANCE), 
                LiftState.reverseSubstationCollect, new StateTransition(ELEVATOR_REV_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_REV_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_REV_DOUBLE_SUB_ANGLE), REV_DOUBLE_SUB_PLAN, LiftState.reverseSubstationCollect, ELEVATOR_REV_DOUBLE_SUB_TOLERANCE, ARM_REV_DOUBLE_SUB_TOLERANCE, WRIST_REV_DOUBLE_SUB_TOLERANCE))),

            Map.entry(LiftState.stowed, Map.of(
                LiftState.stowed, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.transition, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.groundCube, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.groundCone, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.midCubeScore, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.midConeScore, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.highCubeScore, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.highConeScore, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE),
                LiftState.reverseSubstationCollect, new StateTransition(ELEVATOR_TRANSITION_POS, Rotation2d.fromDegrees(ARM_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_TRANSITION_ANGLE), STOWED_PLAN, LiftState.transition, ELEVATOR_TRANSITION_TOLERANCE, ARM_TRANSITION_TOLERANCE, WRIST_TRANSITION_TOLERANCE ))), 

            Map.entry(LiftState.groundCone, defaultTable), 
            Map.entry(LiftState.groundCube, defaultTable),
            Map.entry(LiftState.midConeScore, safeCollectTable),
            Map.entry(LiftState.midCubeScore, safeCollectTable),
            Map.entry(LiftState.highCubeScore, safeCollectTable),
            Map.entry(LiftState.highConeScore, safeCollectTable),
            
            Map.entry(LiftState.doubleSubstationCollect, defaultTable), 
            Map.entry(LiftState.reverseSubstationCollect, defaultTable),
            Map.entry(LiftState.safeCollect, defaultTable));

    /**
     * Gets the state transition from the current state to the goal state
     * 
     * @param current The current state
     * @param goal The goal state
     * 
     * @return The state transition
     */
    public static StateTransition get(LiftState current, LiftState goal) {
        return stateTable.get(current).get(goal);        
    }
}
