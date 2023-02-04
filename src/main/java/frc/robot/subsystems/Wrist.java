package frc.robot.subsystems;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    Path gridlockFile = Paths.get("home/lvuser/gridlock");
    Path blackoutFile = Paths.get("home/lvuser/blackout");

    public Wrist() {
        motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, WristConstants.MOTOR_INVERT, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
        wristController = NeoConfig.createPIDController(motor.getPIDController(), WristConstants.kP, WristConstants.kI, WristConstants.kD);
        encoder = NeoConfig.createAbsoluteEncoder(motor, WristConstants.ENCODER_INVERT, OFFSET);

        if (Files.exists(blackoutFile)) {
            OFFSET = WristConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            OFFSET = WristConstants.ENCODER_OFFSET_GRIDLOCK;
        }
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition()).plus(Rotation2d.fromDegrees(OFFSET));
    }

    public void setAngle(Rotation2d angle) { 
        angle = angle.minus(Rotation2d.fromDegrees(OFFSET));
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
