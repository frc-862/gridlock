// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDController extends SubsystemBase {

  //initialize variables
  int ledPort = 9;
  final int ledLength = 37;
  int waitTime = 10;
  long currentTime = System.currentTimeMillis();
  long nextEventTime = System.currentTimeMillis();
  long timeCounter;

  //create LEDs
  AddressableLED led = new AddressableLED(ledPort);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
  String prevState = "none";
  String currentState = "none";

  //set up an LED set
  public LEDController() {
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    start();

    orangeAndBlue();
  }

  public void readyCollect(){
    //green
    setFullStripColor(0, 255, 0, 0.75f);
    currentState = "readyCollect";
    led.setData(ledBuffer);
  }

  public void hasGamePiece(){
    //blue
    setFullStripColor(0, 0, 255, 0.75f);
    currentState = "hasGamePiece";
    led.setData(ledBuffer);
  }

  public void readyScore(){
    //pink
    setFullStripColor(230, 50, 200, 0.75f);
    currentState = "readyScore";
    led.setData(ledBuffer);
  }

  public void readyDrop(){
    //cyan
    setFullStripColor(96, 209, 149, 0.75f);
    currentState = "readyDrop";
    led.setData(ledBuffer);
  }

  public void believeScored(){
    //red
    setFullStripColor(255, 0, 0, 0.75f);
    currentState = "believeScored";
    led.setData(ledBuffer);
  }

  public void wantsCone(){
    //yellow
    setFullStripColor(255, 230, 20, 0.75f);
    currentState = "wantsCone";
    led.setData(ledBuffer);
  }
  
  public void wantsCube(){
    //purple
    setFullStripColor(220, 30, 240, 0.75f);
    currentState = "wantsCube";
    led.setData(ledBuffer);
  }

  public void fullWhite(){
    //white
    setFullStripColor(255, 255, 255, 0.75f);
    currentState = "fullWhite";
    led.setData(ledBuffer);
  }

  public void blink(){
    //flashes between blue and orange
    if ((System.currentTimeMillis() % 1000) < 500){
      setFullStripColor(0, 0, 255, 0.75f);
    } else {
      setFullStripColor(255, 125, 15, 0.75f);
    }

    currentState = "blink";
    led.setData(ledBuffer);
  }

  public void orangeAndBlue(){
    //switches between blue and orange
    if ((System.currentTimeMillis() % 1000) < 500){
      for(int i = 0; i < ledLength; i += 2){
        ledBuffer.setRGB(i, (int) (255 * .75), 
                      (int) (125 * .75), 
                      (int) (15 * .75));
      }
      for(int i = 1; i < ledLength; i += 2){
        ledBuffer.setRGB(i, (int) (15 * .75), 
                      (int) (70 * .75), 
                      (int) (255 * .75));
      }
    } else {
      for(int i = 1; i < ledLength; i += 2){
        ledBuffer.setRGB(i, (int) (255 * .75), 
                      (int) (125 * .75), 
                      (int) (15 * .75));
      }
      for(int i = 0; i < ledLength; i += 2){
        ledBuffer.setRGB(i, (int) (15 * .75), 
                      (int) (70 * .75), 
                      (int) (255 * .75));
      }
    }

    currentState = "orangeAndBlue";
    led.setData(ledBuffer);
  }

  //turn off LEDs
  public void stop(){
    led.stop();
  }

  //turn on LEDs
  public void start(){
    led.start();
  }

  @Override
  public void periodic() {
    prevState = currentState;
    currentTime = System.currentTimeMillis();
    
    //  compare current time to the next event time to take action and set next event
    if (nextEventTime-currentTime <= 0){
      if (currentState == "blink"){
        blink();
      }
      if (currentState == "orangeAndBlue"){
        orangeAndBlue();
      }
      led.setData(ledBuffer);
      nextEventTime = currentTime + 100;
    }
  }

  /**
   * lets you set the full strip color and brightness
   * 
   * @param r value 0-255
   * @param g value 0-255
   * @param b value 0-255
   * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half, etc.
   */
  private void setFullStripColor (int r, int g, int b, float brightness){
    for(int i = 0; i < ledLength; i++){
      ledBuffer.setRGB(i, (int) (r * brightness), 
                      (int) (g * brightness), 
                      (int) (b * brightness));
    } 
  }
}