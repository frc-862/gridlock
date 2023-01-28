package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.WristConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;

  private PIDController wristController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
  public Wrist() {
    motor = NeoConfig.createMotor(CAN.WRIST_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake); 
  }
  public double getDegrees() {
    return motor.getEncoder().getPosition() / WristConstants.TICKS * WristConstants.GEAR_RATIO * 360;
  }

  public void setAngle(double target) { 
    setPower(wristController.calculate(getDegrees(), LightningMath.inputModulus(target, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE)));
  }

  public void setGains(double kP, double kI, double kD) {
    wristController = new PIDController(kP, kI, kD);
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
