// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.config.NeoConfig;

public class Collector extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  public Collector() {
    leftMotor = NeoConfig.createMotor(CAN.LEFT_COLLECTOR_MOTOR, false, 0, 0, MotorType.kBrushless, IdleMode.kCoast);
    rightMotor = NeoConfig.createMotor(CAN.RIGHT_COLLECTOR_MOTOR, true, 0, 0, MotorType.kBrushless, IdleMode.kCoast);
  }

  public void runCollector(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    
  }
}
