package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LEDController extends SubsystemBase {

    // initialize variables
    private int ledPort = 9;
    private final int ledLength = 37;
    private long currentTime = System.currentTimeMillis();
    private long nextEventTime = System.currentTimeMillis();
    private long timeCounter;

    // create LEDs
    private AddressableLED led = new AddressableLED(ledPort);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
    private String prevState = "none";
    private String currentState = "none";


    // set up an LED set
    public LEDController() {
        led.setLength(ledBuffer.getLength());

        // Set the data
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

    public void orangeAndBlue() {
        // switches between blue and orange
        if ((System.currentTimeMillis() % 1000) < 500) {
            for (int i = 0; i < ledLength; i += 2) {
                ledBuffer.setRGB(i, (int) (255 * .75), (int) (125 * .75), (int) (15 * .75));
            }
            for (int i = 1; i < ledLength; i += 2) {
                ledBuffer.setRGB(i, (int) (15 * .75), (int) (70 * .75), (int) (255 * .75));
            }
        } else {
            for (int i = 1; i < ledLength; i += 2) {
                ledBuffer.setRGB(i, (int) (255 * .75), (int) (125 * .75), (int) (15 * .75));
            }
            for (int i = 0; i < ledLength; i += 2) {
                ledBuffer.setRGB(i, (int) (15 * .75), (int) (70 * .75), (int) (255 * .75));
            }
        }

        currentState = "orangeAndBlue";
        led.setData(ledBuffer);
    }
    
    public void readyDrop() {
        // cyan
        setFullStripColor(96, 209, 149, 0.75f);
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
        setFullStripColor(255, 230, 20, 0.75f);
        currentState = "wantsCone";
        led.setData(ledBuffer);
    }

    public void wantsCube() {
        // purple
        setFullStripColor(220, 30, 240, 0.75f);
        currentState = "wantsCube";
        led.setData(ledBuffer);
    }
    
    public void readyScore() {
        // flashes between blue and orange
        if ((System.currentTimeMillis() % 1000) < 500) {
            setFullStripColor(0, 0, 255, 0.75f);
        } else {
            setFullStripColor(255, 125, 15, 0.75f);
        }

        currentState = "readyScore";
        led.setData(ledBuffer);
    }

    //rainbow
    public void autoAligned(){
        if ((System.currentTimeMillis() % 1000) < 500) {
            //red
            for (int i = 0; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
            //orange
            for (int i = 1; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, (int) (255 * .75), (int) (153 * .75), 0);
            }
            //yellow
            for (int i = 2; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, (int) (255 * .75), (int) (230 * .75), (int) (20 * .75));
            }
            //green
            for (int i = 3; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, 0, (int) (255 * .75), 0);
            }
            //blue
            for (int i = 4; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, 0, 0, (int) (255 * .75));
            }
            //indigo
            for (int i = 5; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, (int) (75 * .75), 0, (int) (130 * .75));
            }
            //violet
            for (int i = 6; i < ledLength; i += 7) {
                ledBuffer.setRGB(i, (int) (143 * .75), 0, (int) (255 * .75));
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
        prevState = currentState;
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

    private void initDashboard() {
        // ShuffleBoard button setup
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