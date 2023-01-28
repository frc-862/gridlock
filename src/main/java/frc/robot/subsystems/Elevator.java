// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Elevator extends SubsystemBase {
  private CANSparkMax motor;

  private PIDController elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  public Elevator() {
    motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);    
    
  }

  public double getHeight() {
    return motor.getEncoder().getPosition() / ElevatorConstants.TICKS * ElevatorConstants.INCHES_PER_REV * ElevatorConstants.GEAR_RATIO;
  }

  public void setHeight(double target) {
    setPower(elevatorController.calculate(getHeight(), LightningMath.inputModulus(target, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT)));
  }

  public void setGains(double kP, double kI, double kD) {
    elevatorController = new PIDController(kP, kI, kD);
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
