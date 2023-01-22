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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.LightningContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;

public class RobotContainer extends LightningContainer {

  // Creates new LED contorller
  private static final LEDController led = new LEDController();
  
    // Creates our drivetrain subsystem
    private static final Drivetrain drivetrain = new Drivetrain();

    // Creates our driver controller and deadzone
    private static final XboxController driver = new XboxController(0);
    private static final JoystickFilter joystickFilter =
            new JoystickFilter(XboxControllerConstants.DEADBAND, XboxControllerConstants.MIN_POWER,
                    XboxControllerConstants.MAX_POWER, Mode.CUBED);

    private static final AutonomousCommandFactory autoFactory =
            new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry,
                    drivetrain.getDriveKinematics(), DrivetrainConstants.DRIVE_PID_CONSTANTS,
                    DrivetrainConstants.THETA_PID_CONSTANTS, drivetrain::setStates, drivetrain);

    // Configure the button bindings
    @Override
    protected void configureButtonBindings() {
        // Back button to reset feild centeric driving to current heading of the robot
        new Trigger(driver::getBackButton)
                .onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {


    }

    @Override
    protected void configureDefaultCommands() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX()),
                        () -> joystickFilter.filter(driver.getLeftY()),
                        () -> -joystickFilter.filter(driver.getRightX())));

    }

    @Override
    protected void configureSystemTests() {}

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {
        ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardTab ledTab = Shuffleboard.getTab("leds");

    }

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}
}
