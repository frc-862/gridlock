package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Collect;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.ServoTurn;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {

    /**
     * Moves Servo and Auto Balances Prints out all commands for 1 piece
     * 
     * @param drivetrain
     * @param servoturn
     * @return HashMap of all paths for 1 piece
     */
    public static HashMap<String, Command> getPathMap1Piece(Drivetrain drivetrain, ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo", new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }

    /**
     * Moves Servo and Auto Balances Runs Lift and Collector for 2nd piece
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @return
     */
    public static HashMap<String, Command> getPathMap2Piece(Drivetrain drivetrain, ServoTurn servoturn, Lift lift, Collector collector) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo", new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Set-Groud-Collect", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Collect-Game-Piece", new Collect(collector, () -> -.5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Set-Ground-Score", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Score-Game-Piece", new Collect(collector, () -> .5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving-2", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }

    /**
     * Moves Servo and Auto Balances Runs Lift and Collector for 2nd and 3rd piece
     * 
     * @param drivetrain
     * @param servoturn
     * @param lift
     * @param collector
     * @return
     */
    public static HashMap<String, Command> getPathMapPiece(Drivetrain drivetrain, ServoTurn servoturn, Lift lift, Collector collector) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo", new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Set-Groud-Collect", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Collect-Game-Piece", new Collect(collector, () -> -.5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Set-Ground-Score", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Score-Game-Piece", new Collect(collector, () -> .5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving-2", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Set-Groud-Collect-2", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Collect-Game-Piece-2", new Collect(collector, () -> -.5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving-3", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Set-Ground-Score-2", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        eventMap.put("Score-Game-Piece-2", new Collect(collector, () -> .5d).until(() -> collector.hasPiece()));
        eventMap.put("Store-For-Moving-4", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }
}
