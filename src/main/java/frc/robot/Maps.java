package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.commands.CubeAlign;
import frc.robot.commands.HoldPower;
import frc.robot.commands.ThrowCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.ServoTurn;
import frc.thunder.vision.VisionBase;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * The general Hash map for all paths. Has most calls needed for the paths to run.
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @param leds
     * @return
     */
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain, ServoTurn servoturn, Lift lift, Collector collector, LEDs leds, Arm arm, LimelightFront frontLimelight, int cycle) {
        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Score-Piece-Servo", new InstantCommand(servoturn::flickServo, servoturn)); // Triggers the servo to drop the cube at the start
        // eventMap.put("Ground-Collect-Cone", new RunCommand(() -> lift.setGoalState(LiftState.groundCone), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("Ground-Collect-Cube", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("High-Score-Cone", new RunCommand(() -> lift.setGoalState(LiftState.highConeScore), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("High-Score-Cube", new RunCommand(() -> lift.setGoalState(LiftState.highCubeScore), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("High-Score-Cube-Back", new RunCommand(() -> lift.setGoalState(LiftState.OTB_High), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("Mid-Score-Cube-Back", new RunCommand(() -> lift.setGoalState(LiftState.OTB_Mid), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("Stow", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached)); // Call to state table
        // eventMap.put("Instant-Sync", new InstantCommand(() -> drivetrain.instantSyncVision())); // Not curently used but can sync vision position to odometry position
        // eventMap.put("Stop-Collect", new InstantCommand(() -> collector.stop(), collector)); // Stops the collector
        // eventMap.put("Collect", new InstantCommand(() -> collector.setPower(1d))); // Starts the collector intake
        // eventMap.put("Hold-Power", new InstantCommand(() -> collector.setPower(CollectorConstants.HOLD_POWER_CUBE))); // Power to hold cube once piece is collected
        // eventMap.put("Score-Slow", new InstantCommand(() -> collector.setPower(-.50))); //Lower power for no roll out
        eventMap.put("Auto-Align-Cube", new CubeAlign(drivetrain, frontLimelight, servoturn, lift, leds, arm, collector, cycle));
        // eventMap.put("Score", new InstantCommand(() -> collector.setPower(-1d))); // Full power Score to spit out cube
        // eventMap.put("Auto-Balance", new AutoBalance(drivetrain)); // Call to AutoBalance command requires drivetrain, must be at the end of the path
        // eventMap.put("Throw-Cube", new ThrowCube(lift, arm, collector).withTimeout(4)); // Throw cube behind the robot, timeout is to guarnatee the robot drives even if the arm fails
        // eventMap.put("Turn-On-Vision", new InstantCommand(() -> VisionBase.enableVision())); // Turns on vision
        // eventMap.put("Turn-Off-Vision", new InstantCommand(() -> VisionBase.disableVision())); // Turns off vision
        // eventMap.put("Pos1", new InstantCommand(() -> drivetrain.setAprilTagTarget(1))); // Sets the isolation to pos 1, tags 8 and 1  CABLE
        // eventMap.put("Pos2", new InstantCommand(() -> drivetrain.setAprilTagTarget(2))); // Sets the isolation to pos 2, tags 7 and 2  MIDDLE
        // eventMap.put("Pos3", new InstantCommand(() -> drivetrain.setAprilTagTarget(3))); // Sets the isolation to pos 3, tags 6 and 3  OPEN
        return eventMap;
    }
}
