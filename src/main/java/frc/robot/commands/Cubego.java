// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;

public class Cubego extends CommandBase {

    Drivetrain drivetrain;
    LimelightFront limelightFront;

    //Set up PIDs
    PIDController cubePIDy = new PIDController(0,0,0);
    PIDController cubePIDz = new PIDController(0,0,0);



    /** Creates a new Cubego. */
  public Cubego(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //cube detect is in 3(I hope)
    limelightFront.setPipelineNum(3);

    //TODO tune these 
    cubePIDy.setSetpoint(0);
    cubePIDz.setSetpoint(0);

    //sets up tolerance 
    cubePIDy.setTolerance(0);
    cubePIDz.setTolerance(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelightFront.hasVision()){ // If sees object
        //if the cube is within 5 degrees for y and z planes
        if(Math.abs(limelightFront.getVerticalOffset()) > Constants.AutoAlignConstants.TOLERANCE_CUBE_ALLIGN_Y && Math.abs(limelightFront.getHorizontalOffset()) > Constants.AutoAlignConstants.TOLERANCE_CUBE_ALLIGN_Z){
            drivetrain.drive(new ChassisSpeeds(0,cubePIDy.calculate(limelightFront.getVerticalOffset()),cubePIDz.calculate(limelightFront.getHorizontalOffset())));

        }

    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   limelightFront.setPipelineNum(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
