package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.thunder.shuffleboard.LightningShuffleboard;

/**
 * The servo subsystem
 */
public class ServoTurn extends SubsystemBase {
    Servo servo = new Servo(RobotMap.PWM.SERVO);

    public ServoTurn() {}

    /**
     * Turns the servo to the specified position
     * 
     * @param position the position to turn the servo to, between 0 and 1
     */

    public void turnServo(double position) {
        servo.set(position);
        // LightningShuffleboard.setDouble("Servo", "Servo Position", position);
    }
}
