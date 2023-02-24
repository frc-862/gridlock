package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap.*;
import frc.thunder.config.NeoConfig;
import frc.thunder.logging.DataLogger;

public class Collector extends SubsystemBase {
    private CANSparkMax motor;
    private ColorSensorV3 colorSensor;
    private final ColorMatch colorMatch;

    public enum GamePiece {
        CONE, CUBE, NONE
    }

    public Collector() {
        motor = NeoConfig.createMotor(CAN.LEFT_COLLECTOR_MOTOR, CollectorConstants.MOTOR_INVERT, CollectorConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE,
                MotorType.kBrushless, IdleMode.kCoast);
        colorSensor = new ColorSensorV3(i2c.COLOR_SENSOR);

        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(CollectorConstants.CONE_OPTIMAL);
        colorMatch.addColorMatch(CollectorConstants.CUBE_OPTIMAL);

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Collector Motor Temperature",
                () -> motor.getMotorTemperature());
        DataLogger.addDataElement("Collector Motor Controller Input Voltage",
                () -> motor.getBusVoltage());
        DataLogger.addDataElement("Collector Motor Controller Output (Amps)",
                () -> motor.getOutputCurrent());
        DataLogger.addDataElement("Collector Motor Controller Output (Volts)",
                () -> motor.getAppliedOutput());
        DataLogger.addDataElement("Color Sensor Raw Color", () -> colorSensor.getColor().toString());
        DataLogger.addDataElement("Color Sensor Detected Game Piece", () -> getGamePiece().toString());
        DataLogger.addDataElement("Color Sensor Confidence", () -> getConfidence());

    }

    /**
     * getGamePiece
     * 
     * @return the game piece detected by the color sensor (Either CUBE, CONE, or NONE)
     */
    public GamePiece getGamePiece() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        if (match.color == CollectorConstants.CUBE_OPTIMAL) {
            return GamePiece.CUBE;
        } else if (match.color == CollectorConstants.CONE_OPTIMAL) {
            return GamePiece.CONE;
        } else {
            return GamePiece.NONE;
        }
    }

    /**
     * getConfidence
     * @return the confidence of the color sensor as a decimal (0 - 1)
     */
    public double getConfidence() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        return match.confidence;
    }

    /**
     * hasPiece
     * @return true if the color sensor detects a game piece
     */
    public boolean hasPiece() {
        return getGamePiece() != GamePiece.NONE;
    }

    /**
     * setPower
     * @param power the percent speed to set the elevator motor to
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * stop
     * Sets the power of the collector motor to 0
     */
    public void stop() {
        setPower(0d);
    }
}
