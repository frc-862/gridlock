package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Colors;

public class LEDController extends SubsystemBase {

    // initialize variables
    private final int ledPort = 9;
    private final int ledLength = 37;
    private long currentTime = System.currentTimeMillis();
    private long nextEventTime = System.currentTimeMillis();

    // create LEDs
    private AddressableLED led = new AddressableLED(ledPort);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
    private String currentState = "none";

    // set up an LED set
    public LEDController() {
        led.setLength(ledBuffer.getLength());

        // turn the strip off on initalization
        setFullStripColor(0, 0, 0, 0.75f);
        led.setData(ledBuffer);
        start();

        orangeAndBlue();
        initDashboard();
    }

    public void readyCollect() {
        // green
        setFullStripColor(0, 255, 0, 0.75f);
        currentState = "readyCollect";
        led.setData(ledBuffer);
    }
    
    public void fullWhite() {
        // white
        setFullStripColor(255, 255, 255, 0.75f);
        currentState = "fullWhite";
        led.setData(ledBuffer);
    }

    //sets every other LED to orange or blue, and switches them every 0.5 seconds
    public void orangeAndBlue() {
        if ((System.currentTimeMillis() % 1000) < 500) {
            for (int i = 0; i < ledLength; i += 2) {
                setRGBFromArray(i, Colors.lightningOrange, 0.75f);
            }
            for (int i = 1; i < ledLength; i += 2) {
                setRGBFromArray(i, Colors.lightningBlue, 0.75f);
            }
        } else {
            for (int i = 1; i < ledLength; i += 2) {
                setRGBFromArray(i, Colors.lightningOrange, 0.75f);
            }
            for (int i = 0; i < ledLength; i += 2) {
                setRGBFromArray(i, Colors.lightningBlue, 0.75f);
            }
        }

        currentState = "orangeAndBlue";
        led.setData(ledBuffer);
    }
    
    public void readyDrop() {
        // cyan
        setFullStripColor(Colors.cyan, 0.75f);
        currentState = "readyDrop";
        led.setData(ledBuffer);
    }

    public void believeScored() {
        // red
        setFullStripColor(255, 0, 0, 0.75f);
        currentState = "believeScored";
        led.setData(ledBuffer);
    }

    public void hasGamePiece() {
        // flashes green
        if ((System.currentTimeMillis() % 1000) < 500) {
            setFullStripColor(0, 255, 0, 0.75f);
        } else {
            setFullStripColor(0, 0, 0, 0.75f);
        }
        currentState = "hasGamePiece";
        led.setData(ledBuffer);
    }

    public void wantsCone() {
        // yellow
        setFullStripColor(Colors.yellow, 0.75f);
        currentState = "wantsCone";
        led.setData(ledBuffer);
    }

    public void wantsCube() {
        // purple
        setFullStripColor(Colors.purple, 0.75f);
        currentState = "wantsCube";
        led.setData(ledBuffer);
    }
    
    public void readyScore() {
        // flashes between blue and orange
        if ((System.currentTimeMillis() % 1000) < 500) {
            setFullStripColor(Colors.lightningBlue, 0.75f);
        } else {
            setFullStripColor(Colors.lightningOrange, 0.75f);
        }

        currentState = "readyScore";
        led.setData(ledBuffer);
    }

    public void autoAligned(){
        //flash a rainbow pattern on and off
        if ((System.currentTimeMillis() % 1000) < 500) {
            //how many times the rainbow will repeat across the strip
            float rainbowRepetitions  = 3f;

            for (int i = 0; i < ledLength; i += 1) {
                int hValue = (int) ((((float) i / (float) (ledLength / rainbowRepetitions)) % 1) * 180);
                ledBuffer.setHSV(i, hValue, 255, 255);
            }
        } else {
            setFullStripColor(0, 0, 0, 0f);
        }
        
        currentState = "autoAligned";
        led.setData(ledBuffer);
    }

    // turn off LEDs
    public void stop() {
        led.stop();
    }

    // turn on LEDs
    public void start() {
        led.start();
    }

    @Override
    public void periodic() {
        currentTime = System.currentTimeMillis();

        // compare current time to the next event time to take action and set next event
        if (nextEventTime - currentTime <= 0) {
            if (currentState == "readyScore") {
                readyScore();
            } else if (currentState == "orangeAndBlue") {
                orangeAndBlue();
            } else if (currentState == "hasGamePiece") {
                hasGamePiece();
            } else if (currentState == "autoAligned") {
                autoAligned();
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
     * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half,
     *        etc.
     */
    private void setFullStripColor(int r, int g, int b, float brightness) {
        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setRGB(i, (int) (r * brightness), (int) (g * brightness),
                    (int) (b * brightness));
        }
    }

    /**
     * lets you set the full strip color and brightness
     * 
     * @param rgbArray array of an r, g, and b value (integers 0-255)
     * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half,
     *        etc.
     */
    private void setFullStripColor(int [] rgbArray, float brightness) {
        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setRGB(i, (int) (rgbArray[0] * brightness), (int) (rgbArray[1] * brightness),
                    (int) (rgbArray[2] * brightness));
        }
    }

    /**
     * lets you set an LED to a color with an array of r, g, and b
     * 
     * @param index the index to write
     * @param rgbArray array of an r, g, and b value (integers 0-255)
     * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half,
     *        etc.
     */
    private void setRGBFromArray(int index, int [] rgbArray, float brightness) {
        ledBuffer.setRGB(index, (int) (rgbArray[0] * brightness), (int) (rgbArray[1] * brightness),
             (int) (rgbArray[2] * brightness));
    }

    /**
     * initializes the commands and buttons on ShuffleBoard
     */
    private void initDashboard() {
        var ledTab = Shuffleboard.getTab("LEDs");
        if (led != null) {
            //strategy chosen methods
            ledTab.add("hasGamePiece", new InstantCommand(this::hasGamePiece, this));
            ledTab.add("wantsCone", new InstantCommand(this::wantsCone, this));
            ledTab.add("wantsCube", new InstantCommand(this::wantsCube, this));
            ledTab.add("readyScore", new InstantCommand(this::readyScore, this));
            ledTab.add("autoAligned", new InstantCommand(this::autoAligned, this));

            //others
            ledTab.add("stop", new InstantCommand(this::stop, this));
            ledTab.add("start", new InstantCommand(this::start, this));
            ledTab.add("readyCollect", new InstantCommand(this::readyCollect, this));
            ledTab.add("readyDrop", new InstantCommand(this::readyDrop, this));
            ledTab.add("believeScored", new InstantCommand(this::believeScored, this));
            ledTab.add("fullWhite", new InstantCommand(this::fullWhite, this));
            ledTab.add("orangeAndBlue", new InstantCommand(this::orangeAndBlue, this));
        }
    }
}