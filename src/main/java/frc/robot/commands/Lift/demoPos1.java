// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class demoPos1 extends SequentialCommandGroup {
  public demoPos1(Elevator elevator, Arm arm, Wrist wrist) {
    addCommands(
        new RunCommand(() -> elevator.setExtension(5), elevator).until(elevator::onTarget),
        new ParallelCommandGroup(
            new RunCommand(() -> arm.setAngle(Rotation2d.fromDegrees(-70)), arm),
            new RunCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(-90)), wrist)
        )
    );
  }
}
