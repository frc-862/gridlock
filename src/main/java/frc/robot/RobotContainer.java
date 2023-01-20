// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDController;
import frc.thunder.LightningContainer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends LightningContainer {
  XboxController controller = new XboxController(0);
  LEDController led; //  = new LEDController();

  @Override
  protected void configureButtonBindings() {
    var tab = Shuffleboard.getTab("leds");
    led = new LEDController();

    if(led != null) {
      tab.add(new InstantCommand(led::readyCollect, led));
      tab.add(new InstantCommand(led::hasGamePiece, led));
      tab.add(new InstantCommand(led::readyScore, led));
      tab.add(new InstantCommand(led::readyDrop, led));
      tab.add(new InstantCommand(led::believeScored, led));
      tab.add(new InstantCommand(led::wantsCone, led));
      tab.add(new InstantCommand(led::wantsCube, led));
      tab.add(new InstantCommand(led::fullWhite, led));
      tab.add(new InstantCommand(led::blink, led));
    }
  }

  @Override
  protected void configureSystemTests() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void configureDefaultCommands() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void releaseDefaultCommands() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void initializeDashboardCommands() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void configureAutonomousCommands() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void configureFaultCodes() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void configureFaultMonitors() {
    // TODO Auto-generated method stub

  }

}
