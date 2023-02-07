package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ArmConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController armController;

    public Arm() {
        motor = NeoConfig.createMotor(CAN.ARM_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
        armController = NeoConfig.createPIDController(motor.getPIDController(), ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setAngle(double target) {
        target = LightningMath.inputModulus(target, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        armController.setReference(target / 360 / ArmConstants.GEAR_RATIO * ArmConstants.TICKS,
                                   CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() {
        return motor.getEncoder().getPosition() / ArmConstants.TICKS * ArmConstants.GEAR_RATIO * 360;
    }
    public void setGains(double kP, double kI, double kD) {
        armController = NeoConfig.createPIDController(armController, kP, kI, kD);
    }

    public void setPower(double speed){
        motor.set(speed);
    }
    
    public void stop(){
        setPower(0);
    }     

    @Override
    public void periodic() {
     
    }
}
