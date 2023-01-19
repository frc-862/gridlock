// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;

public class FourBar extends SubsystemBase {
  private CANSparkMax motor;

  private PIDController fourBarController = new PIDController(0, 0, 0);
  public FourBar() {
    motor = NeoConfig.createMotor(CAN.FOURBAR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake); 
  }

  public void setAngle(double target) {
    motor.set(fourBarController.calculate(motor.getEncoder().getPosition(), target));
  }

  public void setGains(double kP, double kI, double kD) {
    fourBarController = new PIDController(kP, kI, kD);
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
