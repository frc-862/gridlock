// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class safeDeploy extends SequentialCommandGroup {
  public safeDeploy(Elevator elevator, Arm arm) {
    addCommands(
        new RunCommand(() -> elevator.setExtension(4), elevator).until(elevator::onTarget),
        new RunCommand(() -> arm.setAngle(Rotation2d.fromDegrees(-83)), arm)
    );
  }
}
