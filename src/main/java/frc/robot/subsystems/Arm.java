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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    Path gridlockFile = Paths.get("home/lvuser/gridlock");
    Path blackoutFile = Paths.get("home/lvuser/blackout");

    public Arm() {
        motor = NeoConfig.createMotor(CAN.ARM_MOTOR, ArmConstants.MOTOR_INVERT, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
        controller = NeoConfig.createPIDController(motor.getPIDController(), ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        encoder = NeoConfig.createAbsoluteEncoder(motor, ArmConstants.ENCODER_INVERT, OFFSET);

        if (Files.exists(blackoutFile)) {
            OFFSET = ArmConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            OFFSET = ArmConstants.ENCODER_OFFSET_GRIDLOCK;
        }
    }

    public void setAngle(Rotation2d angle) {
        angle = angle.minus(Rotation2d.fromDegrees(OFFSET));
        targetAngle = LightningMath.inputModulus(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        controller.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition()).plus(Rotation2d.fromDegrees(OFFSET));
    }
    public void setGains(double kP, double kI, double kD) {
        controller = NeoConfig.createPIDController(controller, kP, kI, kD);
    }

    public void setPower(double speed){
        motor.set(speed);
    }
    
    public void stop(){
        setPower(0);
    }     

    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < ArmConstants.TOLERANCE;
    }

    public Translation2d getArmXY() {
        return new Translation2d(ArmConstants.LENGTH, getAngle());
    }
}
