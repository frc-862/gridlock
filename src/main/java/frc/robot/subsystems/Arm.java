package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.math.LightningMath;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    public Arm() {
        if (Constants.isBlackout()) {
            OFFSET = ArmConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            OFFSET = ArmConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        motor = NeoConfig.createMotor(
            CAN.ARM_MOTOR,
            ArmConstants.MOTOR_INVERT,
            ArmConstants.CURRENT_LIMIT,
            Constants.VOLTAGE_COMP_VOLTAGE,
            ArmConstants.MOTOR_TYPE,
            IdleMode.kBrake
        );
        controller = NeoConfig.createPIDController(
            motor.getPIDController(),
            new SparkMaxPIDGains(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kF));
        encoder = NeoConfig.createAbsoluteEncoder(motor, ArmConstants.ENCODER_INVERT, OFFSET);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setAngle(Rotation2d angle) {
        targetAngle = LightningMath.inputModulus(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        controller.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }
    public void setGains(double kP, double kI, double kD, double kF) {
        controller = NeoConfig.createPIDController(controller, new SparkMaxPIDGains(kP, kI, kD, kF));
    }

    public void setPower(double speed){
        motor.set(speed);
    }
    
    public void stop(){
        setPower(0d);
    }     

    public void setOffset(double offset) {
        //Theory is to use offset to account for arm movement
        OFFSET = offset;
        encoder.setZeroOffset(offset);
    }

    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < ArmConstants.TOLERANCE;
    }

    public Translation2d getArmXY() {
        return new Translation2d(ArmConstants.LENGTH, getAngle());
    }
}
