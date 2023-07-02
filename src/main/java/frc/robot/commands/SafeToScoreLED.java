package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Collector.GamePiece;

public class SafeToScoreLED extends CommandBase {
    private LEDs leds;
    private Drivetrain drivetrain;
    private Collector collector;

    public SafeToScoreLED(LEDs leds, Drivetrain drivetrain, Collector collector) {
        this.leds = leds;
        this.drivetrain = drivetrain;
        this.collector = collector;
        addRequirements(leds);
    }

    @Override
    public void execute() {
        double position = drivetrain.getPose().getX();
        GamePiece currPiece = collector.getGamePiece();
        
        if(Math.abs(position - 2.4) < .25){
            leds.setColor(Color.kDarkRed);
        } else{
            leds.wantsPiece(currPiece);
        }
    }
}
