package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LedConstants.Colors;

public class LEDController extends SubsystemBase {

    private long currentTime = System.currentTimeMillis();
    private long nextEventTime = System.currentTimeMillis();

    // create LEDs
    private AddressableLED led = new AddressableLED(LedConstants.port);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.length);
    private String currentState = "none";

    private int swirlPosition = 0;

    // set up an LED set
    public LEDController() {
        led.setLength(ledBuffer.getLength());
        // turn the strip off on initalization
        setFullStripColor(Colors.off, LedConstants.brightness);
        led.setData(ledBuffer);
        start();

        orangeAndBlue();
        initDashboard();
    }

    public void readyCollect() {
        // green
        setFullStripColor(Colors.green, LedConstants.brightness);
        currentState = "readyCollect";
        led.setData(ledBuffer);
    }
    
    public void fullWhite() {
        // white
        setFullStripColor(Colors.white, LedConstants.brightness);
        currentState = "fullWhite";
        led.setData(ledBuffer);
    }

    public void swirl() {
        //swirl pattern wooooooo (swirls orange and blue)
        int swirlLength = (LedConstants.length/2);
        setFullStripColor(Colors.lightningBlue, LedConstants.brightness);

        for (int i = 0; i < swirlLength; i++){
            setRGBFromArray((i + swirlPosition) % LedConstants.length, Colors.lightningOrange, LedConstants.brightness);
        }

        swirlPosition += 1;

        currentState = "swirl";
        led.setData(ledBuffer);
    }

    //sets every other LED to orange or blue, and switches them every 0.5 seconds
    public void orangeAndBlue() {
        if ((System.currentTimeMillis() % 1000) < 500) {
            for (int i = 0; i < LedConstants.length; i += 2) {
                setRGBFromArray(i, Colors.lightningOrange, LedConstants.brightness);
            }
            for (int i = 1; i < LedConstants.length; i += 2) {
                setRGBFromArray(i, Colors.lightningBlue, LedConstants.brightness);
            }
        } else {
            for (int i = 1; i < LedConstants.length; i += 2) {
                setRGBFromArray(i, Colors.lightningOrange, LedConstants.brightness);
            }
            for (int i = 0; i < LedConstants.length; i += 2) {
                setRGBFromArray(i, Colors.lightningBlue, LedConstants.brightness);
            }
        }

        currentState = "orangeAndBlue";
        led.setData(ledBuffer);
    }
    
    public void readyDrop() {
        // cyan
        setFullStripColor(Colors.cyan, LedConstants.brightness);
        currentState = "readyDrop";
        led.setData(ledBuffer);
    }

    public void believeScored() {
        // red
        setFullStripColor(Colors.red, LedConstants.brightness);
        currentState = "believeScored";
        led.setData(ledBuffer);
    }

    public void hasGamePiece() {
        // flashes green
        if ((System.currentTimeMillis() % 1000) < 500) {
            setFullStripColor(Colors.green, LedConstants.brightness);
        } else {
            setFullStripColor(Colors.off, LedConstants.brightness);
        }
        currentState = "hasGamePiece";
        led.setData(ledBuffer);
    }

    public void wantsCone() {
        // yellow
        setFullStripColor(Colors.yellow, LedConstants.brightness);
        currentState = "wantsCone";
        led.setData(ledBuffer);
    }

    public void wantsCube() {
        // purple
        setFullStripColor(Colors.purple, LedConstants.brightness);
        currentState = "wantsCube";
        led.setData(ledBuffer);
    }
    
    public void readyScore() {
        // flashes between blue and orange
        if ((System.currentTimeMillis() % 1000) < 500) {
            setFullStripColor(Colors.lightningBlue, LedConstants.brightness);
        } else {
            setFullStripColor(Colors.lightningOrange, LedConstants.brightness);
        }

        currentState = "readyScore";
        led.setData(ledBuffer);
    }

    public void autoAligned(){
        //flash a rainbow pattern on and off
        if ((System.currentTimeMillis() % 1000) < 500) {
            //how many times the rainbow will repeat across the strip
            double rainbowRepetitions  = 3f;

            for (int i = 0; i < LedConstants.length; i += 1) {
                int hValue = (int) ((((double) i / (double) (LedConstants.length / rainbowRepetitions)) % 1) * 180);
                ledBuffer.setHSV(i, hValue, 255, 255);
            }
        } else {
            setFullStripColor(Colors.off, 0f);
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
            } else if (currentState == "swirl"){
                swirl();
            }

            led.setData(ledBuffer);
            nextEventTime = currentTime + 100;
        }
    }

    /**
     * lets you set the full strip color and brightness
     * 
     * @param rgbArray array of an r, g, and b value (integers 0-255)
     * @param brightness decimal value for brightness where 1.0 is full brightness, 0.5 is half,
     *        etc.
     */
    private void setFullStripColor(int [] rgbArray, double brightness) {
        for (int i = 0; i < LedConstants.length; i++) {
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
    private void setRGBFromArray(int index, int [] rgbArray, double brightness) {
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
            ledTab.add("swirl", new InstantCommand(this::swirl, this));

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