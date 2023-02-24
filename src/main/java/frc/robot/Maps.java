package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

    public static HashMap<String, Command> getPath1StartBMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Auto-Balance", new AutoBalance(drivetrain));
        return eventMap;
    }

    public static HashMap<String, Command> getPath2StartBMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        return eventMap;
    }

    public static HashMap<String, Command> getPath3StartAMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        return eventMap;
    }

    public static HashMap<String, Command> getPath4StartAMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath4StartAChargeMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }

    public static HashMap<String, Command> getPath5StartCMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        // eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath6StartCMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath6ChargeMap(Drivetrain drivetrain,
            ServoTurn servoturn, Lift lift, Collector collector) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Set-Groud-Collect",
                new RunCommand(() -> lift.setGoalState(LiftState.ground), lift).until(lift::goalReached));
        eventMap.put("Collect-Game-Piece",
                new Collect(collector, () -> -.5d).until(() -> collector.isPieceCollected()));
        eventMap.put("Store-For-Moving",
                new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Set-Ground-Score",
                new RunCommand(() -> lift.setGoalState(LiftState.ground), lift).until(lift::goalReached));
        eventMap.put("Score-Game-Piece",
                new Collect(collector, () -> .5d).until(() -> collector.isPieceCollected()));
        eventMap.put("Store-For-Moving-2",
                new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }

    public static HashMap<String, Command> getPath7StartAMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece-One", new PrintCommand("Collect-Game-Piece-One"));
        eventMap.put("Score-Game-Piece-One", new PrintCommand("Score-Game-Piece-One"));
        eventMap.put("Collect-Game-Piece-Two", new PrintCommand("Collect-Game-Piece-Two"));
        eventMap.put("Score-Game-Piece-Two", new PrintCommand("Score-Game-Piece-Two"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath8StartCMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece-One", new PrintCommand("Collect-Game-Piece-One"));
        eventMap.put("Score-Game-Piece-One", new PrintCommand("Score-Game-Piece-One"));
        eventMap.put("Collect-Game-Piece-Two", new PrintCommand("Collect-Game-Piece-Two"));
        eventMap.put("Score-Game-Piece-Two", new PrintCommand("Score-Game-Piece-Two"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath9StartBMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        return eventMap;
    }

    public static HashMap<String, Command> getPath9StartBChargeMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }

}
