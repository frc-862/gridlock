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

public class Elevator extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private PIDController elevatorController = new PIDController(0, 0, 0);
  public Elevator() {
    leftMotor = NeoConfig.createMotor(CAN.LEFT_ELEVATOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);
    rightMotor = NeoConfig.createMotor(CAN.RIGHT_ELEVATOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);    
    
    leftMotor.follow(rightMotor);
  }

  public void setPosition(double target) {
    rightMotor.set(elevatorController.calculate(rightMotor.getEncoder().getPosition(), target));
  }

  public void setGains(double kP, double kI, double kD) {
    elevatorController = new PIDController(kP, kI, kD);
  }

  public void setPower(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void stop(){
    setPower(0);
  }

  @Override
  public void periodic() {
   
  }
}
