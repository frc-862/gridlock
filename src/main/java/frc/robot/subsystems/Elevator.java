package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Elevator extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder encoder;

    public Elevator() {
        motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);        
        elevatorController = NeoConfig.createPIDController(motor.getPIDController(), ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD); 
        encoder = motor.getEncoder(); //TODO: add this to thunder's neoconfig (with inverts etc.)
    }

    public double getHeight() {
        return encoder.getPosition() / ElevatorConstants.TICKS * ElevatorConstants.INCHES_PER_REV * ElevatorConstants.GEAR_RATIO;
    }

    public void setHeight(double target) {
        target = LightningMath.inputModulus(target / ElevatorConstants.INCHES_PER_REV / ElevatorConstants.GEAR_RATIO * ElevatorConstants.TICKS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
        elevatorController.setReference(target, CANSparkMax.ControlType.kPosition);
    }

    public void setGains(double kP, double kI, double kD) {
        elevatorController = NeoConfig.createPIDController(elevatorController, kP, kI, kD);
    }

    public void setPower(double speed){
        motor.set(speed);
    }

    public void stop(){
        setPower(0d);
    }

    public boolean onTarget() {
        return false; //TODO figure out how to do this properly
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(getHeight(), ElevatorConstants.ANGLE);
    }
}
