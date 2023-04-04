package frc.robot.commands.Lift;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.Range;

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

    //SAFE ZONES

    private static final double ELEVATOR_MAX_EXTENSION = ElevatorConstants.MAX_EXTENSION;
    private static final double ARM_MAX_ANGLE = ArmConstants.MAX_ANGLE;
    private static final double WRIST_MAX_ANGLE = WristConstants.MAX_ANGLE;

    private static final double ELEVATOR_MIN_EXTENSION = ElevatorConstants.MIN_EXTENSION;
    private static final double ARM_MIN_ANGLE = ArmConstants.MIN_ANGLE;
    private static final double WRIST_MIN_ANGLE = WristConstants.MIN_ANGLE;

    private static final double ELEVATOR_STOW_SAFE = 8d;
    private static final double ARM_STOW_SAFE = -105d;
    private static final double WRIST_STOW_SAFE = 95;

    private static final double WRIST_SCORE_TO_COLLECT_SAFE = 0d;

    private static final double ELEVATOR_STOWED_POS = 2;
    private static final double ARM_STOWED_ANGLE = -112;
    private static final double WRIST_STOWED_ANGLE = 112;
    private static final LiftPlan STOWED_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_STOW_COLLECT_TRANSITION_POS = 8;
    private static final double ARM_STOW_COLLECT_TRANSITION_ANGLE = -70;
    private static final double WRIST_STOW_COLLECT_TRANSITION_ANGLE = 90;

    private static final double ELEVATOR_GROUND_CONE_POS = 0;
    private static final double ARM_GROUND_CONE_ANGLE = -70.5;
    private static final double WRIST_GROUND_CONE_ANGLE = 38d;
    private static final LiftPlan GROUND_CONE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_GROUND_CUBE_POS = 5.5;
    private static final double ARM_GROUND_CUBE_ANGLE = -70.5;
    private static final double WRIST_GROUND_CUBE_ANGLE = 22;
    private static final LiftPlan GROUND_CUBE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_MID_CUBE_POS = 0;
    private static final double ARM_MID_CUBE_ANGLE = 0;
    private static final double WRIST_MID_CUBE_ANGLE = -6d;
    private static final LiftPlan MID_CUBE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_MID_CONE_POS = 6;
    private static final double ARM_MID_CONE_ANGLE = 16.1;
    private static final double WRIST_MID_CONE_ANGLE = -90d;
    private static final LiftPlan MID_CONE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_HIGH_CUBE_POS = 10d;
    private static final double ARM_HIGH_CUBE_ANGLE = 6d;
    private static final double WRIST_HIGH_CUBE_ANGLE = -6d;
    private static final LiftPlan HIGH_CUBE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_HIGH_CONE_POS = 24;
    private static final double ARM_HIGH_CONE_ANGLE = 10d;
    private static final double WRIST_HIGH_CONE_ANGLE = -69d;
    private static final LiftPlan HIGH_CONE_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_DOUBLE_SUB_POS = 17.1;
    private static final double ARM_DOUBLE_SUB_ANGLE = 14.4d;
    private static final double WRIST_DOUBLE_SUB_ANGLE = -59d;
    private static final LiftPlan DOUBLE_SUB_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_SINGLE_SUB_POS = 6.3d;
    private static final double ARM_SINGLE_SUB_ANGLE = -66d;
    private static final double WRIST_SINGLE_SUB_ANGLE = 119d;
    private static final LiftPlan SINGLE_SUB_PLAN = LiftPlan.parallel;

    private static final double ELEVATOR_DEPLOYED_POS = 8;
    private static final double ARM_DEPLOYED_ANGLE = -70;

    // A map of all possible state transitions
    private static Map<LiftState, StateTransition> defaultTable = Map.of(
            LiftState.stowed, new StateTransition(ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.armAndWristThenEle, LiftState.stowed),
            LiftState.elevatorDeployed, new StateTransition(ELEVATOR_DEPLOYED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.eleAndWristThenArm, LiftState.elevatorDeployed),

            LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CUBE_ANGLE), GROUND_CUBE_PLAN, LiftState.groundCube),
            LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CONE_ANGLE), GROUND_CONE_PLAN, LiftState.groundCone),

            LiftState.midCubeScore, new StateTransition(ELEVATOR_MID_CUBE_POS, Rotation2d.fromDegrees(ARM_MID_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CUBE_ANGLE), MID_CUBE_PLAN, LiftState.midCubeScore),
            LiftState.midConeScore, new StateTransition(ELEVATOR_MID_CONE_POS, Rotation2d.fromDegrees(ARM_MID_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CONE_ANGLE), MID_CONE_PLAN, LiftState.midConeScore),
            LiftState.highCubeScore, new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), HIGH_CUBE_PLAN, LiftState.highCubeScore),
            LiftState.highConeScore, new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), HIGH_CONE_PLAN, LiftState.highConeScore),
            LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_DOUBLE_SUB_ANGLE), DOUBLE_SUB_PLAN, LiftState.doubleSubstationCollect),
            LiftState.singleSubstationCollect, new StateTransition(ELEVATOR_SINGLE_SUB_POS, Rotation2d.fromDegrees(ARM_SINGLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_SINGLE_SUB_ANGLE), SINGLE_SUB_PLAN, LiftState.singleSubstationCollect));

    private static Map<LiftState, StateTransition> scoreTable = new HashMap<LiftState, StateTransition>();
    private static Map<LiftState, StateTransition> groundTable = new HashMap<LiftState, StateTransition>();
    private static Map<LiftState, StateTransition> stowedTable = new HashMap<LiftState, StateTransition>();
    private static Map<LiftState, StateTransition> stowCollectTable = new HashMap<LiftState, StateTransition>();
    private static Map<LiftState, StateTransition> stowScoreTable = new HashMap<LiftState, StateTransition>();
    private static Map<LiftState, StateTransition> scoreToCollectTable = new HashMap<LiftState, StateTransition>();

    static {
        scoreTable.putAll(defaultTable);
            scoreTable.replace(LiftState.stowed, new StateTransition(ELEVATOR_DEPLOYED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.eleAndWristThenArm, LiftState.stowedScore, Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));
            scoreTable.replace(LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(112), GROUND_CUBE_PLAN, LiftState.scoreToCollect, Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            scoreTable.replace(LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(112), GROUND_CONE_PLAN, LiftState.scoreToCollect, Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));

        groundTable.putAll(defaultTable);
            groundTable.replace(LiftState.stowed, new StateTransition(ELEVATOR_STOW_COLLECT_TRANSITION_POS, Rotation2d.fromDegrees(ARM_STOW_COLLECT_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_STOW_COLLECT_TRANSITION_ANGLE), LiftPlan.parallel, LiftState.stowedCollect, Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));
            groundTable.replace(LiftState.elevatorDeployed, new StateTransition(ELEVATOR_STOW_COLLECT_TRANSITION_POS, Rotation2d.fromDegrees(ARM_STOW_COLLECT_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_STOW_COLLECT_TRANSITION_ANGLE), LiftPlan.parallel, LiftState.elevatorDeployed, Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));


        stowedTable.putAll(defaultTable);
            stowedTable.replace(LiftState.stowed,                  new StateTransition(ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), STOWED_PLAN, LiftState.stowed));
            stowedTable.replace(LiftState.groundCube,              new StateTransition(ELEVATOR_STOW_COLLECT_TRANSITION_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_STOW_COLLECT_TRANSITION_ANGLE), LiftPlan.eleArmWrist, LiftState.stowedCollect,             Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(-90d, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.groundCone,              new StateTransition(ELEVATOR_STOW_COLLECT_TRANSITION_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_STOW_COLLECT_TRANSITION_ANGLE), LiftPlan.eleArmWrist, LiftState.stowedCollect,             Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(-90d, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.highCubeScore,           new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), LiftPlan.eleArmWrist, LiftState.highCubeScore,                                           Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_DEPLOYED_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.highConeScore,           new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), LiftPlan.eleArmWrist, LiftState.highConeScore,                                           Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_DEPLOYED_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.doubleSubstationCollect, new StateTransition(ELEVATOR_DOUBLE_SUB_POS, Rotation2d.fromDegrees(ARM_DOUBLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_DOUBLE_SUB_ANGLE), LiftPlan.eleArmWrist, LiftState.doubleSubstationCollect,                              Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_DEPLOYED_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.singleSubstationCollect, new StateTransition(ELEVATOR_DEPLOYED_POS, Rotation2d.fromDegrees(ARM_SINGLE_SUB_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.eleArmWrist, LiftState.stowedScore,                                                Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(-90d, ARM_MAX_ANGLE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowedTable.replace(LiftState.midCubeScore,            new StateTransition(ELEVATOR_DEPLOYED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.parallel, LiftState.stowedScore));
            stowedTable.replace(LiftState.midConeScore,            new StateTransition(ELEVATOR_DEPLOYED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.parallel, LiftState.stowedScore));
            
        stowCollectTable.putAll(defaultTable);
            stowCollectTable.replace(LiftState.stowed, new StateTransition(ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.armAndWristThenEle, LiftState.stowed, Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_STOW_SAFE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));
            stowCollectTable.replace(LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CUBE_ANGLE), GROUND_CUBE_PLAN, LiftState.groundCube));
            stowCollectTable.replace(LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CONE_ANGLE), GROUND_CONE_PLAN, LiftState.groundCone));

        stowScoreTable.putAll(defaultTable);
            stowScoreTable.replace(LiftState.stowed, new StateTransition(ELEVATOR_STOWED_POS, Rotation2d.fromDegrees(ARM_STOWED_ANGLE), Rotation2d.fromDegrees(WRIST_STOWED_ANGLE), LiftPlan.armAndWristThenEle, LiftState.stowed, Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_STOW_SAFE), Range.between(WRIST_STOW_SAFE, WRIST_MAX_ANGLE)));

            stowScoreTable.replace(LiftState.groundCube, new StateTransition(ELEVATOR_GROUND_CUBE_POS, Rotation2d.fromDegrees(ARM_GROUND_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CUBE_ANGLE), GROUND_CUBE_PLAN, LiftState.groundCube));
            stowScoreTable.replace(LiftState.groundCone, new StateTransition(ELEVATOR_GROUND_CONE_POS, Rotation2d.fromDegrees(ARM_GROUND_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_GROUND_CONE_ANGLE), GROUND_CONE_PLAN, LiftState.groundCone));

            stowScoreTable.replace(LiftState.midCubeScore, new StateTransition(ELEVATOR_MID_CUBE_POS, Rotation2d.fromDegrees(ARM_MID_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CUBE_ANGLE), LiftPlan.armThenWristAndEle, LiftState.midCubeScore,      Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_DEPLOYED_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));
            stowScoreTable.replace(LiftState.midConeScore, new StateTransition(ELEVATOR_MID_CONE_POS, Rotation2d.fromDegrees(ARM_MID_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_MID_CONE_ANGLE), LiftPlan.armThenWristAndEle, LiftState.midConeScore,      Range.between(ELEVATOR_MIN_EXTENSION, ELEVATOR_MAX_EXTENSION), Range.between(ARM_DEPLOYED_ANGLE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));
            stowScoreTable.replace(LiftState.highCubeScore, new StateTransition(ELEVATOR_HIGH_CUBE_POS, Rotation2d.fromDegrees(ARM_HIGH_CUBE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CUBE_ANGLE), LiftPlan.armAndWristThenEle, LiftState.highCubeScore, Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_STOW_SAFE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));
            stowScoreTable.replace(LiftState.highConeScore, new StateTransition(ELEVATOR_HIGH_CONE_POS, Rotation2d.fromDegrees(ARM_HIGH_CONE_ANGLE), Rotation2d.fromDegrees(WRIST_HIGH_CONE_ANGLE), LiftPlan.armAndWristThenEle, LiftState.highConeScore, Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_STOW_SAFE, ARM_MAX_ANGLE), Range.between(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE)));

        scoreToCollectTable.putAll(defaultTable);
            scoreToCollectTable.replace(LiftState.stowed, new StateTransition(ELEVATOR_STOW_COLLECT_TRANSITION_POS, Rotation2d.fromDegrees(ARM_STOW_COLLECT_TRANSITION_ANGLE), Rotation2d.fromDegrees(WRIST_STOW_COLLECT_TRANSITION_ANGLE), STOWED_PLAN, LiftState.stowedCollect, Range.between(ELEVATOR_STOW_SAFE, ELEVATOR_MAX_EXTENSION), Range.between(ARM_MIN_ANGLE, ARM_STOW_SAFE), Range.between(WRIST_SCORE_TO_COLLECT_SAFE, WRIST_MAX_ANGLE)));
    }

    // Index is current state, goal state; outputs a StateTransition
    private static Map<LiftState, Map<LiftState, StateTransition>> stateTable = Map.ofEntries(
            Map.entry(LiftState.stowed, stowedTable),
            Map.entry(LiftState.groundCone, groundTable), 
            Map.entry(LiftState.groundCube, groundTable),

            Map.entry(LiftState.midConeScore, scoreTable), 
            Map.entry(LiftState.midCubeScore, scoreTable), 
            Map.entry(LiftState.highCubeScore, scoreTable),
            Map.entry(LiftState.highConeScore, scoreTable),

            Map.entry(LiftState.doubleSubstationCollect, scoreTable), 
            Map.entry(LiftState.singleSubstationCollect, scoreTable),

            Map.entry(LiftState.stowedCollect, stowCollectTable), 
            Map.entry(LiftState.stowedScore, stowScoreTable), 
            Map.entry(LiftState.scoreToCollect, stowCollectTable),
            Map.entry(LiftState.elevatorDeployed, defaultTable));
            

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
