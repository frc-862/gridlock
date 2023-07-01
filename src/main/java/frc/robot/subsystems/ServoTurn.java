package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The servo subsystem
 */
public class ServoTurn extends SubsystemBase {
    private Servo servo = new Servo(RobotMap.PWM.SERVO);
    private double position = AutonomousConstants.SERVO_DOWN;

    private LightningShuffleboardPeriodic periodicShuffleboard;

    private boolean flickServo = false; //TODO: make this a command (i didn't because i didn't want to have to edit the paths)
    private double flickServoTime = Timer.getFPGATimestamp(); //TODO: make this a command (i didn't because i didn't want to have to edit the paths)

    public ServoTurn() {
        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();
    }

    // Initializes the shuffleboard values and starts logging data   
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("ServoTurn", 2, 
        new Pair<String, Object>("Servo Position", (DoubleSupplier) () -> position));
    }

    /**
     * Turns the servo to the specified position
     * 
     * @param position the position to turn the servo to, between 0 and 1
     */

    public void turnServo(double position) {
        this.position = position;
        servo.set(position);
    }

    public void flickServo() {
        flickServo = true;
        flickServoTime = Timer.getFPGATimestamp();
        turnServo(AutonomousConstants.SERVO_UP);
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            periodicShuffleboard.loop();
        }

        if (flickServo) {
            if (Timer.getFPGATimestamp() - flickServoTime > 2) {
                turnServo(AutonomousConstants.SERVO_DOWN);
                flickServo = false;
            }
        }
    }
}
