package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ServoTurn;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {
    
    public static HashMap<String, Command> getPath6ChargeMap(Drivetrain drivetrain, ServoTurn servoTurn) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score-Game-Piece-Servo", new InstantCommand(() -> servoTurn.turnServo(.25), servoTurn));
        eventMap.put("Collect-Game-Piece", new PrintCommand("Collect-Game-Piece"));
        eventMap.put("Score-Game-Piece", new PrintCommand("Score-Game-Piece"));
        eventMap.put("Auto-balance", new AutoBalance(drivetrain));
        return eventMap;
    }
}
