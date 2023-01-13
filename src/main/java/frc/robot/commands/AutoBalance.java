// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  private Drivetrain drivetrain;
  private PIDController pid = new PIDController(Constants.AB_KP, Constants.AB_KI, Constants.AB_KD);
  
  public AutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(Math.abs((drivetrain.getRoll().getDegrees() + drivetrain.getPitch().getDegrees())/2) > Constants.AB_MAX_AVERAGE_DEVIATION) { //maybe add check for theoretical color sensor?
      drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(pid.calculate(drivetrain.getGravityVector()[0], 0)),
       												drivetrain.percentOutputToMetersPerSecond(pid.calculate(drivetrain.getGravityVector()[1], 0)), 
													drivetrain.percentOutputToMetersPerSecond(0)));
    } else {
		drivetrain.stop();
	}
    
    
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
