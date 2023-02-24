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
    //Use for all paths 
    public static HashMap<String, Command> getPathMapLift(Drivetrain drivetrain,
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
    
    //use when lift isn't supposed to move
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain,
            ServoTurn servoturn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo",
                new InstantCommand(() -> servoturn.turnServo(.25), servoturn));
        eventMap.put("Set-Groud-Collect", new PrintCommand("Set-Groud-Collect"));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Store-For-Moving", new PrintCommand("Store-For-Moving"));
        eventMap.put("Set-Ground-Score", new PrintCommand("Set-Ground-Score"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        eventMap.put("Store-For-Moving-2", new PrintCommand("Store-For-Moving-2"));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }
}
