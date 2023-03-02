package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

/**
 * LED subsystem
 */
public class LEDs extends SubsystemBase {
    /** Creates a new LED subsystem. */
    // Establish the led variables
    private static int orangeSwirlPosition = 0;
    private static int blueSwirlPosition = (LedConstants.ledLength / 2) + 1;
    private static int swirlLength = (LedConstants.ledLength / 2);

    // Create LEDs
    private CANdle leds = new CANdle(LedConstants.ledPort, "Canivore");
    private CANdleConfiguration ledConfig = new CANdleConfiguration();
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, LedConstants.ledSpeed, LedConstants.ledLength);
    String currentState = "none";

    /*
     * Initialize a CANdle for the robot
     */
    public LEDs() {
        ledConfig.stripType = LEDStripType.RGB;
        ledConfig.brightnessScalar = 0.5;
        leds.configAllSettings(ledConfig);
        off();
        initDashboard();
    }

    /**
     * Swirls orange and blue
     */
    public void swirl() {
        // swirl pattern wooooooo (swirls orange and blue)
        leds.setLEDs(0, 0, 255);
        for (int i = 0; i < swirlLength; i++) {
            leds.setLEDs(255, 75, 0, 0, ((i + orangeSwirlPosition) % LedConstants.ledLength), 1);
        }
        for (int i = 0; i < swirlLength; i++) {
            leds.setLEDs(0, 0, 255, 0, ((i + blueSwirlPosition) % LedConstants.ledLength), 1);
        }
        orangeSwirlPosition += 1;
        blueSwirlPosition += 1;
        currentState = "swirl";
    }

    /**
     * Flashes between blue and orange
     */
    public void readyScore() {
        if ((System.currentTimeMillis() % 1000) < 500) {
            leds.setLEDs(0, 0, 255);
        } else {
            leds.setLEDs(255, 75, 0);
        }

        currentState = "readyScore";
    }

    /**
     * Sets LEDS to Purple
     */
    public void wantsCube() {
        leds.setLEDs(220, 30, 240);
        currentState = "wantsCube";
    }

    /**
     * Sets LEDS to Yellow
     */
    public void wantsCone() {
        leds.setLEDs(255, 230, 20);
        currentState = "wantsCone";
    }

    /**
     * Flashes leds green
     */
    public void hasGamePiece() {
        if ((System.currentTimeMillis() % 1000) < 500) {
            leds.setLEDs(0, 255, 0);
        } else {
            off();
        }
        currentState = "hasGamePiece";
    }

    /**
     * Turns off leds
     */
    public void off() {
        leds.setLEDs(0, 0, 0);
    }

    public void orangeAndBlue() {

        currentState = "orangeAndBlue";

        if ((System.currentTimeMillis() % 1000) < 500) {

            for (int i = 0; i % 2 == 0 && i <= LedConstants.ledLength; i += 2) {
                leds.setLEDs(255, 75, 0, 0, i, 1);
            }
            for (int x = 1; x % 2 != 0 && x <= LedConstants.ledLength; x += 2) {
                leds.setLEDs(0, 0, 255, 0, x, 1);
            }
        } else if ((System.currentTimeMillis() % 1000) > 500) {

            for (int i = 0; i % 2 == 0 && i <= LedConstants.ledLength; i += 2) {
                leds.setLEDs(0, 0, 255, 0, i, 1);
            }
            for (int x = 1; x % 2 != 0 && x <= LedConstants.ledLength; x += 2) {
                leds.setLEDs(255, 75, 0, 0, x, 1);
            }

        }

    }

    /**
     * Sets full strip to white
     */
    public void fullStripWhite() {
        for (int i = 0; i < LedConstants.ledLength; i += 1) {
            leds.setLEDs(255, 255, 255, 0, i, 1);
        }
    }

    /**
     * Sets LEDS to rainbow
     */
    public void autoAligned() {
        RainbowAnimation rainbowAnim = new RainbowAnimation(0, LedConstants.ledSpeed, LedConstants.ledLength);
        leds.animate(rainbowAnim);
        currentState = "autoAligned";
    }

    @Override
    public void periodic() {
        //State logic
        if (currentState == "autoAligned") {
            rainbowAnim = new RainbowAnimation(1, LedConstants.ledSpeed, LedConstants.ledLength);
            leds.animate(rainbowAnim);
        } else {
            rainbowAnim = new RainbowAnimation(0, 1, 0);
            leds.animate(rainbowAnim);
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
        var ledTab = Shuffleboard.getTab("LEDs");
        ledTab.add("hasGamePiece", new InstantCommand(this::hasGamePiece, this));
        ledTab.add("wantsCone", new InstantCommand(this::wantsCone, this));
        ledTab.add("wantsCube", new InstantCommand(this::wantsCube, this));
        ledTab.add("readyScore", new InstantCommand(this::readyScore, this));
        ledTab.add("autoAligned", new InstantCommand(this::autoAligned, this));
        ledTab.add("swirl", new InstantCommand(this::swirl, this));
        ledTab.add("stop", new InstantCommand(this::off, this));
        ledTab.add("fullWhite", new InstantCommand(this::fullStripWhite, this));
        ledTab.add("orangeAndBlue", new InstantCommand(this::orangeAndBlue, this));
    }
}
