package frc.robot;

import java.util.HashMap;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.commands.HoldPower;
import frc.robot.commands.ThrowCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.ServoTurn;
import frc.thunder.command.core.TimedCommand;
import frc.thunder.vision.VisionBase;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * The general Hash map for all paths. Has most calls needed for the paths to run. NEED TO FIX score
     * piece
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @param leds
     * @return
     */
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain, ServoTurn servoturn, Lift lift, Collector collector, LEDs leds, Arm arm) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Piece-Servo", new InstantCommand(servoturn::flickServo, servoturn));
        eventMap.put("Ground-Collect-Cone", new RunCommand(() -> lift.setGoalState(LiftState.groundCone), lift).until(lift::goalReached));
        eventMap.put("Ground-Collect-Cube", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("High-Score-Cone", new RunCommand(() -> lift.setGoalState(LiftState.highConeScore), lift).until(lift::goalReached));
        eventMap.put("High-Score-Cube", new RunCommand(() -> lift.setGoalState(LiftState.highCubeScore), lift).until(lift::goalReached));
        eventMap.put("High-Score-Cube-Back", new RunCommand(() -> lift.setGoalState(LiftState.OTB_High), lift).until(lift::goalReached));
        eventMap.put("Mid-Score-Cube-Back", new RunCommand(() -> lift.setGoalState(LiftState.OTB_Mid), lift).until(lift::goalReached));
        eventMap.put("Stow", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Instant-Sync", new InstantCommand(() -> drivetrain.instantSyncVision()));
        eventMap.put("Stop-Collect", new InstantCommand(() -> collector.stop(), collector));
        eventMap.put("Collect", new InstantCommand(() -> collector.setPower(1d)));
        eventMap.put("Hold-Power", new InstantCommand(() -> collector.setPower(CollectorConstants.HOLD_POWER_CUBE)));
        eventMap.put("Score-Slow", new InstantCommand(() -> collector.setPower(-.50))); //Lower power for no roll out
        eventMap.put("Score", new InstantCommand(() -> collector.setPower(-1d)));
        eventMap.put("Auto-Balance", new AutoBalance(drivetrain));
        // eventMap.put("Throw-Cube", new ThrowCube(lift, arm, collector).withTimeout(1.5));
        eventMap.put("Throw-Cube", new ThrowCube(lift, arm, collector));
        eventMap.put("Turn-On-Vision", new InstantCommand(() -> VisionBase.enableVision()));
        eventMap.put("Turn-Off-Vision", new InstantCommand(() -> VisionBase.disableVision()));
        eventMap.put("Pos1", new InstantCommand(() -> drivetrain.setAprilTagTarget(1)));
        eventMap.put("Pos2", new InstantCommand(() -> drivetrain.setAprilTagTarget(2)));
        eventMap.put("Pos3", new InstantCommand(() -> drivetrain.setAprilTagTarget(3)));
        // eventMap.put("ResetTag", new InstantCommand(() -> drivetrain.setAprilTagTargetAll()));

        return eventMap;
    }
}
