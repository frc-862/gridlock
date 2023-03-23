package frc.robot.subsystems;

import java.time.Period;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.NeoConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The collector subsystem
 */
public class Collector extends SubsystemBase {
    // The collector motor
    private CANSparkMax motor;

    // The color sensor
    private ColorSensorV3 colorSensor;

    // The rev color matcher
    private final ColorMatch colorMatch;

    // Periodic Shuffleboard
    private LightningShuffleboardPeriodic periodicShuffleboard;

    // Enum of possible game pieces
    public enum GamePiece {
        CONE, CUBE, NONE
    }

    private GamePiece gamePiece = GamePiece.CUBE;

    public Collector() {
        // Create the motor and configure it
        motor = NeoConfig.createMotor(CAN.COLLECTOR_MOTOR, CollectorConstants.MOTOR_INVERT, CollectorConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMPENSATION, CollectorConstants.MOTOR_TYPE,
                CollectorConstants.NEUTRAL_MODE);

        // Create the color sensor
        colorSensor = new ColorSensorV3(i2c.COLOR_SENSOR);

        // Create the color matcher
        colorMatch = new ColorMatch();

        // Add the optimal colors to the color matcher
        colorMatch.addColorMatch(CollectorConstants.CONE_OPTIMAL);
        colorMatch.addColorMatch(CollectorConstants.CUBE_OPTIMAL);

        // Initialize the shuffleboard values and start logging data
        initialiizeShuffleboard();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void runCollector(double power) {
        motor.set(power);
    }

    // Method to start logging
    @SuppressWarnings("unchecked")
    private void initialiizeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Collector", CollectorConstants.LOG_PERIOD,
                // new Pair<String, Object>("Collector motor temperature", (DoubleSupplier) () -> motor.getMotorTemperature()),
                // new Pair<String, Object>("Collector motor controller input voltage", (DoubleSupplier) () -> motor.getBusVoltage()),
                new Pair<String, Object>("Collector motor controller output (amps)", (DoubleSupplier) () -> motor.getOutputCurrent()),
        // new Pair<String, Object>("Collector motor controller output (volts)", (DoubleSupplier) () -> motor.getAppliedOutput()),
        new Pair<String, Object>("Color sensor raw color", (Supplier<Double>) () -> (double) colorSensor.getProximity()),
        new Pair<String, Object>("Color sensor detected game piece", (Supplier<String>) () -> getGamePiece().toString()));
        // new Pair<String, Object>("Color sensor confidence", (DoubleSupplier) () -> getConfidence()));

    }

    /**
     * Gets the game piece detected by the color sensor
     * 
     * @return the game piece detected by the color sensor (Either CUBE, CONE, or NONE)
     */

    public GamePiece getGamePiece() {
        // Color detectedColor = colorSensor.getColor();
        // ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        // if (match.color == CollectorConstants.CUBE_OPTIMAL) {
        //     return GamePiece.CUBE;
        // } else if (match.color == CollectorConstants.CONE_OPTIMAL) {
        //     return GamePiece.CONE;
        // } else {
        //     return GamePiece.NONE;
        // }
        return gamePiece;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    /**
     * Gets the confidence of the color sensor
     * 
     * @return the confidence of the color sensor as a decimal (0 - 1)
     */
    public double getConfidence() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        return match.confidence;
    }

    /**
     * Checks if the color sensor detects a game piece
     * 
     * @return true if the color sensor detects a game piece
     */
    public boolean hasPiece() {
        return true;// getGamePiece() != GamePiece.NONE;
    }

    /**
     * Sets the power of the collector motor
     * 
     * @param power the percent speed to set the elevator motor to
     */
    public void setPower(double power) {
        if(getGamePiece() == GamePiece.CONE) {
            motor.set(MathUtil.clamp(power, -1, 1));
        } else {
            motor.set(power);
        }
            
    }

    /**
     * stop Sets the power of the collector motor to 0
     */
    public void stop() {
        setPower(0d);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Collector", "color sensor blue", colorSensor.getColor().blue);
        LightningShuffleboard.setDouble("Collector", "color sensor green", colorSensor.getColor().green);
        LightningShuffleboard.setDouble("Collector", "color sensor red", colorSensor.getColor().red);

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
        LightningShuffleboard.setBool("Collector", "is cube", match.color == CollectorConstants.CUBE_OPTIMAL);
        LightningShuffleboard.setBool("Collector", "is cone", match.color == CollectorConstants.CONE_OPTIMAL);

        periodicShuffleboard.loop();
    }
}
