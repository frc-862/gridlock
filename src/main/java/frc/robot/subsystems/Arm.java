package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ArmConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;

    public Arm() {
        motor = NeoConfig.createMotor(CAN.ARM_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
        controller = NeoConfig.createPIDController(motor.getPIDController(), ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle); //TODO: add this to thunder's neoconfig
    }

    public void setAngle(Rotation2d angle) {
        double target = LightningMath.inputModulus(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        controller.setReference(target, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() {
        return encoder.getPosition()*360;
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
        return false; //TODO figure out how to do this properly
    }

    public Translation2d getArmXY() {
        return new Translation2d(ArmConstants.LENGTH, new Rotation2d(Math.toRadians(getAngle())));
    }
}
