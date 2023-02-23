package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.logging.DataLogger;

/**
 * The collector subsystem
 */
public class Collector extends SubsystemBase {

    // The collector motor
    private CANSparkMax motor;

    public Collector() {
        // Creates collector motor and configures it
        motor = NeoConfig.createMotor(CAN.LEFT_COLLECTOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kCoast);

        // Starts logging
        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Method to start logging
    private void initLogging() {
        DataLogger.addDataElement("Collector Left Motor Temperature", () -> motor.getMotorTemperature());
        DataLogger.addDataElement("L Collector Motor Controller Input Voltage", () -> motor.getBusVoltage());
        DataLogger.addDataElement("L Collector Motor Controller Output (Amps)", () -> motor.getOutputCurrent());

    }

    /**
     * Runs the collector and the given power
     * 
     * @param power The power to run the collector at
     */
    public void runCollector(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.set(0);
    }
}
