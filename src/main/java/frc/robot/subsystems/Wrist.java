package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;

  private SparkMaxPIDController wristController;
  public Wrist() {
    motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
    wristController = NeoConfig.createPIDController(motor.getPIDController(), WristConstants.kP, WristConstants.kI, WristConstants.kD);
  }
  public double getDegrees() {
    return motor.getEncoder().getPosition() / WristConstants.TICKS * WristConstants.GEAR_RATIO * 360;
  }

  public void setAngle(double target) { 
    target = LightningMath.inputModulus(target / 360 / WristConstants.GEAR_RATIO * WristConstants.TICKS, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE);
    wristController.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  public void setGains(double kP, double kI, double kD) {
    wristController = NeoConfig.createPIDController(wristController, kP, kI, kD);
  }

  public void setPower(double speed){
    motor.set(speed);
  }
  
  public void stop(){
    motor.set(0);
  } 

  @Override
  public void periodic() {
   
  }
}
