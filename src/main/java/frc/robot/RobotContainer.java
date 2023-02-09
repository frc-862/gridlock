package frc.robot;

import frc.robot.subsystems.AprilTagTargetting;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.tests.DriveTrainSystemTest;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.LightningContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private AprilTagTargetting targetting = new AprilTagTargetting();
    // Creates new LED controller
    private static final LEDs underglow = new LEDs();

    private static final Drivetrain drivetrain = new Drivetrain();

    // Creates our driver controller and deadzone
    private static final XboxController driver = new XboxController(0);
    private static final JoystickFilter joystickFilter = new JoystickFilter(XboxControllerConstants.DEADBAND,
            XboxControllerConstants.MIN_POWER,
            XboxControllerConstants.MAX_POWER, Mode.CUBED);

    //creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(
            drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            DrivetrainConstants.DRIVE_PID_CONSTANTS, DrivetrainConstants.THETA_PID_CONSTANTS,
            drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        // Back button to reset field centeric driving to current heading of the robot
        new Trigger(driver::getBackButton)
                .onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle));

        new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        autoFactory.makeTrajectory("Tune", new HashMap<>(),
                new PathConstraints(DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND));
    }

    @Override
    protected void configureDefaultCommands() {
        /* 
         * Set up the default command for the drivetrain.
         * The controls are for field-oriented driving:
         * Left stick Y axis -> forward and backwards movement
         * Left stick X axis -> left and right movement
         * Right stick X axis -> rotation
        */
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX()),
                        () -> joystickFilter.filter(driver.getLeftY()),
                        () -> -joystickFilter.filter(driver.getRightX())));
    }

    @Override
    protected void configureSystemTests() {
        SystemTest.registerTest("fl drive test",
                new DriveTrainSystemTest(drivetrain, drivetrain.getFrontLeftModule(), 0.25));

        SystemTest.registerTest("fr drive test",
                new DriveTrainSystemTest(drivetrain, drivetrain.getFrontRightModule(), 0.25));

        SystemTest.registerTest("bl drive test",
                new DriveTrainSystemTest(drivetrain, drivetrain.getBackLeftModule(), 0.25));

        SystemTest.registerTest("br drive test",
                new DriveTrainSystemTest(drivetrain, drivetrain.getBackRightModule(), 0.25));
    }

    @Override
    protected void releaseDefaultCommands() {
    }

    @Override
    protected void initializeDashboardCommands() {
        ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardTab ledTab = Shuffleboard.getTab("LEDs");
    }

    @Override
    protected void configureFaultCodes() {
    }

    @Override
    protected void configureFaultMonitors() {
    }

    @Override
    protected AutonomousCommandFactory getCommandFactory() {
        return autoFactory;
    }
}
