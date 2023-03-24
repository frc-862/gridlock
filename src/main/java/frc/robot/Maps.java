package frc.robot;

import java.util.HashMap;
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
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.ServoTurn;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * The general Hash map for all paths. 
     * Has most calls needed for the paths to run.
     * NEED TO FIX score piece 
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @param leds
     * @return
     */
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain, Lift lift, ServoTurn servoturn, Collector collector, LEDs leds) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Piece-Servo", new InstantCommand(() -> servoturn.turnServo(AutonomousConstants.SERVO_DEPLOY), servoturn));
        eventMap.put("Ground-Collect-Cone", new RunCommand(() -> lift.setGoalState(LiftState.groundCone), lift).until(lift::goalReached));
        eventMap.put("Ground-Collect-Cube", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("High-Score-Cone", new RunCommand(() -> lift.setGoalState(LiftState.highConeScore), lift).until(lift::goalReached));
        eventMap.put("High-Score-Cube", new RunCommand(() -> lift.setGoalState(LiftState.highCubeScore), lift).until(lift::goalReached));
        eventMap.put("Stow", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));   
        eventMap.put("Stop-Collect", new InstantCommand(() -> collector.stop(), collector));
        eventMap.put("Hold-Power", new InstantCommand(() -> collector.setPower(CollectorConstants.HOLD_POWER), collector));
        eventMap.put("Collect-Piece", new HoldPower(collector, () -> .5d));
        eventMap.put("Score-Piece", new HoldPower(collector, () -> -.5d)); //TODO: switch until to be until no piece
        eventMap.put("Auto-Balance", new AutoBalance(drivetrain));
        eventMap.put("Update-Pos-Vision", new PrintCommand("Update-Pos-Vision")); //TODO add vision code
        return eventMap;
    }
}
