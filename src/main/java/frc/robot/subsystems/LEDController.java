// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;

public class LEDController extends SubsystemBase {

  int ledPort = 9;
  int ledLength = 36;

  AddressableLED led;
  AddressableLEDBuffer ledBuffer;

  /** Creates a new LEDsController. */
  public LEDController() {
    led = new AddressableLED(ledPort);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(ledLength);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  public void readyCollect(){
    for(int i=0; i<=ledLength; i ++){
      ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void hasGamePiece(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 0, 0, 255);
    }
  }

  public void readyScore(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 230, 50, 200);
    }
  }

  public void readyDrop(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 96, 209, 149);
    }
  }
  public void believeScored(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 219, 146, ledLength);
    }
  }

  public void wantsCone(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 255, 230, 30);
    }
  }
  
  public void wantsCube(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 220, 30, 240);
    }
  }

  public void clearLED(){
    for(int i=0; i<=ledLength; i++){
      ledBuffer.setRGB(i, 225, 225, 225);
    }
  }

  public void stop(){
    led.stop();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
      led.setData(ledBuffer);
  }
}
