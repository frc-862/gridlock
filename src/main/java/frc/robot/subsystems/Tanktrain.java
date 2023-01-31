// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tanktrain extends SubsystemBase {

  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  CANSparkMax backLeft; 
  CANSparkMax backRight; 

  /** Creates a new Tanktrain. */

  public Tanktrain() {
    frontLeft = new CANSparkMax(0, null);
    frontRight = new CANSparkMax(0, null);
    backLeft = new CANSparkMax(0, null);
    backRight = new CANSparkMax(0, null);

    //TODO: set inverts, brakemode, etc. (finish this class)
    //TODO: input IMU, inplement functions to get values
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower() {

  }
}
