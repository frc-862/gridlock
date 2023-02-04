package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Wrist extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController wristController;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    public Wrist() {
        if (Constants.isBlackout()) {
            OFFSET = WristConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            OFFSET = WristConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        motor = NeoConfig.createMotor(
            CAN.WRIST_MOTOR,
            WristConstants.MOTOR_INVERT,
            WristConstants.CURRENT_LIMIT,
            Constants.VOLTAGE_COMP_VOLTAGE,
            WristConstants.MOTOR_TYPE,
            WristConstants.NEUTRAL_MODE
        );
        wristController = NeoConfig.createPIDController(
            motor.getPIDController(),
            WristConstants.kP,
            WristConstants.kI,
            WristConstants.kD
        );
        encoder = NeoConfig.createAbsoluteEncoder(motor, WristConstants.ENCODER_INVERT, OFFSET);
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    public void setAngle(Rotation2d angle) { 
        targetAngle = LightningMath.inputModulus(angle.getRotations(), WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
        wristController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public void setGains(double kP, double kI, double kD) {
        wristController = NeoConfig.createPIDController(wristController, kP, kI, kD);
    }

    public void setPower(double speed){
        motor.set(speed);
    }
    
    public void stop(){
        motor.set(0d);
    } 

    public boolean onTarget() {
        return Math.abs(encoder.getPosition() - targetAngle) < WristConstants.TOLERANCE;
    }
}
