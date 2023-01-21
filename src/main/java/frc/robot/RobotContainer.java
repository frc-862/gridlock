// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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
      tab.add("readyCollect", new InstantCommand(led::readyCollect, led));
      tab.add("hasGamePiece", new InstantCommand(led::hasGamePiece, led));
      tab.add("readyScore", new InstantCommand(led::readyScore, led));
      tab.add("readyDrop", new InstantCommand(led::readyDrop, led));
      tab.add("believeScored", new InstantCommand(led::believeScored, led));
      tab.add("wantsCone", new InstantCommand(led::wantsCone, led));
      tab.add("wantsCube", new InstantCommand(led::wantsCube, led));
      tab.add("fullWhite", new InstantCommand(led::fullWhite, led));
      tab.add("blink", new InstantCommand(led::blink, led));
      tab.add("stop", new InstantCommand(led::stop, led));
      tab.add("start", new InstantCommand(led::start, led));
      tab.add("orangeAndBlue", new InstantCommand(led::orangeAndBlue, led));
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
