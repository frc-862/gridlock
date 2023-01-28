package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ArmConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Arm extends SubsystemBase {
  private CANSparkMax motor;

  private PIDController armController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  public Arm() {
    motor = NeoConfig.createMotor(CAN.ARM_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake); 
  }

  public void setAngle(double target) {
    motor.set(armController.calculate(getAngle(), LightningMath.inputModulus(target, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE)));
  }
  public double getAngle() {
    return motor.getEncoder().getPosition() / ArmConstants.TICKS * ArmConstants.GEAR_RATIO * 360;
  }
  public void setGains(double kP, double kI, double kD) {
    armController = new PIDController(kP, kI, kD);
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
