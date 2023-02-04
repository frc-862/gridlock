package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class UnderGlow extends SubsystemBase {
  /** Creates a new UnderGlow. */
  // Establish the led port and length as variables.
  private static final int ledPort = 22;
  private static final int ledLength = 170;
  private static final double ledSpeed = .5;
  private static int swirlPosition = 0;
  private static int swirlLength = (ledLength/2);
  
  // Use them variables!
  private CANdle wunderGlowLeds = new CANdle(ledPort, "Canivore");
  private CANdleConfiguration wunderConfig = new CANdleConfiguration();
  RainbowAnimation rainbowAnim = new RainbowAnimation(1, ledSpeed, ledLength);
  String currentState ="none"; 

  public UnderGlow() {
    wunderConfig.stripType = LEDStripType.RGB;
    wunderConfig.brightnessScalar = 0.5;
    wunderGlowLeds.configAllSettings(wunderConfig);
    off();
    initDashboard();
  }
  
  public void swirl() {
    //swirl pattern wooooooo (swirls orange and blue)
    wunderGlowLeds.setLEDs(0, 0, 255);
    for (int i = 0; i < swirlLength; i++){
        wunderGlowLeds.setLEDs(255, 75, 0, 0, ((i + swirlPosition) % ledLength), swirlLength);
    }
    swirlPosition += 1;
    currentState = "swirl";
  }

  public void readyScore() {
    // flashes between blue and orange
    if ((System.currentTimeMillis() % 1000) < 500) {
        wunderGlowLeds.setLEDs(0, 0, 255);
    } else {
      wunderGlowLeds.setLEDs(255, 75, 0);
    }

    currentState = "readyScore";
}

  public void wantsCube() {
    // yellow
    wunderGlowLeds.setLEDs(220, 30, 240);
    currentState = "wantsCube";
}

  public void wantsCone() {
    // yellow
    wunderGlowLeds.setLEDs(255, 230, 20);
    currentState = "wantsCone";
}

  public void hasGamePiece() {
    // flashes green
    if ((System.currentTimeMillis() % 1000) < 500) {
      wunderGlowLeds.setLEDs(0, 255, 0);
    } else {
        off();
    }
    currentState = "hasGamePiece";
}

  public void off(){
    wunderConfig.brightnessScalar = 0;
    wunderGlowLeds.configAllSettings(wunderConfig);
  }

  public void on(){
    wunderConfig.brightnessScalar = 0;
    wunderGlowLeds.configAllSettings(wunderConfig);
  }

  public void orangeAndBlue() {

    currentState = "orangeAndBlue";
   
    if((System.currentTimeMillis() % 1000) < 500){

      for (int i = 0;  i % 2 == 0 && i <= ledLength ; i += 2) {
        wunderGlowLeds.setLEDs(255, 75, 0, 0, i, 1);
      }
      for (int x = 1;  x % 2 != 0 && x <= ledLength ; x += 2){
        wunderGlowLeds.setLEDs(0, 0, 255, 0, x , 1);
      }
    } else if  ((System.currentTimeMillis() % 1000) > 500){
          
      for (int i = 0;  i % 2 == 0 && i <= ledLength ; i += 2){
        wunderGlowLeds.setLEDs(0, 0, 255, 0, i , 1);
      }
      for (int x = 1; x % 2 != 0 && x <= ledLength; x += 2) {
        wunderGlowLeds.setLEDs(255, 75, 0, 0, x, 1);
      }

      
    }
  
  }

  public void fullStripWhite(){
    for (int i = 0;  i < ledLength ; i += 1) {
      wunderGlowLeds.setLEDs(255, 255, 255, 0, i, 1);
    }
  }

  public void autoAligned () {
    RainbowAnimation rainbowAnim = new RainbowAnimation(0, ledSpeed, ledLength);
    wunderGlowLeds.animate(rainbowAnim);
    currentState = "autoAligned";
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (currentState == "autoAligned") {
      rainbowAnim = new RainbowAnimation(1, ledSpeed, ledLength);
      wunderGlowLeds.animate(rainbowAnim);
    } else {
      rainbowAnim = new RainbowAnimation(0, 1, 0);
      wunderGlowLeds.animate(rainbowAnim);
    }
    if (currentState == "orangeAndBlue") {
      orangeAndBlue();
    }
    if (currentState == "readyScore") {
      readyScore();
    }
    if (currentState == "swirl") {
      swirl();
    }
    if (currentState == "hasGamePiece") {
      hasGamePiece();
    }
  }
  /**
     * initializes the commands and buttons on ShuffleBoard
     */
    private void initDashboard() {
      var underGlowTab = Shuffleboard.getTab("UnderGlow");
      // strategy chosen methods
      underGlowTab.add("hasGamePiece", new InstantCommand(this::hasGamePiece, this));
      underGlowTab.add("wantsCone", new InstantCommand(this::wantsCone, this));
      underGlowTab.add("wantsCube", new InstantCommand(this::wantsCube, this));
      underGlowTab.add("readyScore", new InstantCommand(this::readyScore, this));
      underGlowTab.add("autoAligned", new InstantCommand(this::autoAligned, this));
      underGlowTab.add("swirl", new InstantCommand(this::swirl, this));
      underGlowTab.add("stop", new InstantCommand(this::off, this));
      underGlowTab.add("start", new InstantCommand(this::on, this));
      underGlowTab.add("fullWhite", new InstantCommand(this::fullStripWhite, this));
      underGlowTab.add("orangeAndBlue", new InstantCommand(this::orangeAndBlue, this));
  }
}
