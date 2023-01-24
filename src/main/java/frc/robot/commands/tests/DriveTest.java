// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.swervelib.SwerveModule;

public class DriveTest extends CommandBase {
    /** Creates a new FRDriveTest. */
    private final SwerveModule module;
    private final Drivetrain drivetrain;
    private double driveSpeed = 1;
    private double driveAngle = 0;

    public DriveTest(Drivetrain drivetrain, SwerveModule module, double drivesSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.module = module;
        this.drivetrain = drivetrain;
        this.driveSpeed = drivesSpeed;

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
