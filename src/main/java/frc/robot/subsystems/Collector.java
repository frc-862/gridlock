package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.logging.DataLogger;

public class Collector extends SubsystemBase {
    private CANSparkMax leftMotor;
    // private CANSparkMax rightMotor;

    public Collector() {
        leftMotor = NeoConfig.createMotor(CAN.LEFT_COLLECTOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kCoast);
        // rightMotor = NeoConfig.createMotor(CAN.RIGHT_COLLECTOR_MOTOR, true, 0, 0,
        // MotorType.kBrushless, IdleMode.kCoast);

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Collector Left Motor Temperature", () -> leftMotor.getMotorTemperature());
        // DataLogger.addDataElement("Collector Right Motor Temperature", () ->
        // rightMotor.getMotorTemperature());
        // DataLogger.addDataElement("R Collector Motor Controller Input Voltage", () ->
        // rightMotor.getBusVoltage());
        // DataLogger.addDataElement("R Collector Motor Controller Output (Amps)", () ->
        // rightMotor.getOutputCurrent());
        DataLogger.addDataElement("L Collector Motor Controller Input Voltage", () -> leftMotor.getBusVoltage());
        DataLogger.addDataElement("L Collector Motor Controller Output (Amps)", () -> leftMotor.getOutputCurrent());

    }

    public void runCollector(double power) {
        leftMotor.set(power);
        // rightMotor.set(power);
    }

    public void stop() {
        leftMotor.set(0);
        // rightMotor.set(0);
    }
}
