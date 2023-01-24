// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.swervelib.SwerveModule;

public class TurnTest extends CommandBase {
  /** Creates a new FRDriveTest. */
  private final SwerveModule module;
  private final Drivetrain drivetrain;
  private double driveSpeed = 0;
  private double driveAngle = 0;
  private boolean direction = false;

  public TurnTest(Drivetrain drivetrain, SwerveModule module, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.module = module;
    this.drivetrain = drivetrain;
    this.direction = direction;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.set(drivetrain.velocityToDriveVolts(driveSpeed), driveAngle);
    if (direction) {// If true forward
      driveAngle += 1;
      if ((driveAngle >= 360)) {
        driveAngle -= 360;
      }
    } else if (direction == false) {// If false backward
      driveAngle -= 1;
      if ((driveAngle <= 0)) {
        driveAngle += 360;
      }
    }

    System.out.println("drive angle: " + driveAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
