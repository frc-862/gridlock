// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;
import frc.thunder.math.LightningMath;

public class Elevator extends SubsystemBase {
  private CANSparkMax motor;

  private SparkMaxPIDController elevatorController;
  public Elevator() {
    motor = NeoConfig.createMotor(CAN.ELEVATOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kBrake);    
    elevatorController = NeoConfig.createPIDController(motor.getPIDController(), ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD); 
  }

  public double getHeight() {
    return motor.getEncoder().getPosition() / ElevatorConstants.TICKS * ElevatorConstants.INCHES_PER_REV * ElevatorConstants.GEAR_RATIO;
  }

  public void setHeight(double target) {
    target = LightningMath.inputModulus(target / ElevatorConstants.INCHES_PER_REV / ElevatorConstants.GEAR_RATIO * ElevatorConstants.TICKS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
    elevatorController.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  public void setGains(double kP, double kI, double kD) {
    elevatorController = NeoConfig.createPIDController(elevatorController, kP, kI, kD);
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

  public Translation2d getElevatorXY() {
    return new Translation2d(getHeight(), ElevatorConstants.ANGLE);
}
}
