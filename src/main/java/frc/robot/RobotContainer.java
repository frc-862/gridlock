package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.ManualLift;
import frc.robot.commands.tests.DriveTrainSystemTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.thunder.LightningContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private static final Vision targetting = new Vision();

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain(targetting);
    private static final Arm arm = new Arm();
    private static final Wrist wrist = new Wrist();
    private static final Elevator elevator = new Elevator();

    // Creates new LED controller
    private static final LEDs underglow = new LEDs();

    // Creates our driver controller and deadzones
    private static final XboxController driver =
            new XboxController(XboxControllerConstants.DRIVER_CONTROLLER_PORT);
    private static final JoystickFilter joystickFilter =
            new JoystickFilter(XboxControllerConstants.DEADBAND, XboxControllerConstants.MIN_POWER,
                    XboxControllerConstants.MAX_POWER, Mode.CUBED);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory =
            new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry,
                    drivetrain.getDriveKinematics(), AutonomousConstants.DRIVE_PID_CONSTANTS,
                    AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS,
                    drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        // Back button to reset field centeric driving to current heading of the robot
        new Trigger(driver::getBackButton)
                .onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle));

        new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain));

        new Trigger(driver::getXButton)
                .whileTrue(autoFactory.createManualTrajectory(new PathConstraints(3, 3),
                        drivetrain.getCurrentPathPoint(), autoFactory.makePathPoint(0, 0, 0)));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        autoFactory.makeTrajectory("Meter", new HashMap<>(), new PathConstraints(1, 0.25));
        autoFactory.makeTrajectory("Straight", new HashMap<>(), new PathConstraints(11, 3));
        autoFactory.makeTrajectory("StraightButRotate", new HashMap<>(),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, 1));
        autoFactory.makeTrajectory("7Meter", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path6StartC", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("StraightAndBack", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("StraightAndBackCurve", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("jitter", new HashMap<>(), new PathConstraints(5, 1));
        autoFactory.makeTrajectory("Path8StartC", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
    }

    @Override
    protected void configureDefaultCommands() {
        /*
         * Set up the default command for the drivetrain. The controls are for field-oriented
         * driving: Left stick Y axis -> forward and backwards movement Left stick X axis -> left
         * and right movement Right stick X axis -> rotation
         */
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX()),
                        () -> joystickFilter.filter(driver.getLeftY()),
                        () -> -joystickFilter.filter(driver.getRightX())));

        elevator.setDefaultCommand(
                new ManualLift(() -> driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(),
                        () -> 0, () -> 0, arm, wrist, elevator));
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
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {
        ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardTab ledTab = Shuffleboard.getTab("LEDs");
    }

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected AutonomousCommandFactory getCommandFactory() {
        return autoFactory;
    }
}
