// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StdDevOdo extends CommandBase {

    private Drivetrain drivetrain;

    public StdDevOdo(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Odo X" + drivetrain.getPose().getX());
        System.out.println("Odo Y" + drivetrain.getPose().getY());
        System.out.println("Odo heading" + drivetrain.getPose().getRotation().getRadians());

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
