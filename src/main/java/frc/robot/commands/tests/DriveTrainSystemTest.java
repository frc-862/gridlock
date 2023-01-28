// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.command.core.TimedCommand;
import frc.thunder.swervelib.SwerveModule;


public class DriveTrainSystemTest extends SequentialCommandGroup {
    /** Creates a new DriveTrainSystemTest. */
    public DriveTrainSystemTest(Drivetrain drivetrain, SwerveModule module, double speed) {
        super(new WaitCommand(2), 
                new TimedCommand(new DriveTest(drivetrain, module, speed), 2),
                new WaitCommand(1), 
                new TimedCommand(new DriveTest(drivetrain, module, -speed), 2),
                new WaitCommand(1), 
                new TimedCommand(new TurnTest(drivetrain, module, true), 2),
                new WaitCommand(1), 
                new TimedCommand(new TurnTest(drivetrain, module, false), 2));
    }
}
