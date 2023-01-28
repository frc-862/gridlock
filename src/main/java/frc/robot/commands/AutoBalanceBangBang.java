// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceBangBang extends CommandBase {
  private Drivetrain drivetrain;
  // add gyroscope
  //TODO: initialize tanktrain
  
  public AutoBalanceBangBang(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    double previousAngle = drivetrain.getPitch2d().getDegrees();
                        
    // TODO: tune the value of 0.05 to probably something a lot smaller
    if (Math.abs(drivetrain.getPitch2d().getDegrees() - previousAngle) > 0.05 && drivetrain.getPitch2d().getDegrees() > Constants.OPTIMAL_ROLL) { 
      drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(1), 
                                                    drivetrain.percentOutputToMetersPerSecond(0), 
                                                    drivetrain.percentOutputToMetersPerSecond(0)));
    } else if (Math.abs(drivetrain.getPitch2d().getDegrees() - previousAngle) > 0.05 && drivetrain.getPitch2d().getDegrees() < -Constants.OPTIMAL_ROLL) {
      drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-1), 
                                                    drivetrain.percentOutputToMetersPerSecond(0), 
                                                    drivetrain.percentOutputToMetersPerSecond(0)));
    } else {
      drivetrain.stop();
    }
    // this measures the change in angle and if the robot is actually on a slope
    // if it is, it will move forward until it is level and then it will move back
    
    previousAngle = drivetrain.getPitch2d().getDegrees();
	}
        

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
