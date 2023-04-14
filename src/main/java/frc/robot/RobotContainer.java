package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import edu.wpi.first.math.geometry.Pose2d;

import java.lang.ModuleLayer.Controller;
import java.util.HashMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LimelightFront;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector.GamePiece;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.ServoTurn;
import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.ShuffleBoard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.AutoAlignConstants.SlotPosition;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoScore;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Collect;
import frc.robot.commands.EleUpInCommunity;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.HoldPower;
import frc.robot.commands.RetroLineUp;
import frc.robot.commands.AprilTagLineUp;
import frc.robot.commands.SafeToScoreLED;
import frc.robot.commands.SingleSubstationAlign;
import frc.robot.commands.Lift.DoubleSubstationCollect;
import frc.robot.commands.Lift.Ground;
import frc.robot.commands.Lift.HighScore;
import frc.robot.commands.Lift.MidScore;
import frc.robot.commands.Lift.SingleSubstationCollect;
import frc.robot.commands.Lift.Stow;
import frc.robot.commands.tests.DriveTrainSystemTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.thunder.LightningContainer;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.LimelightConstants;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private static final LimelightFront frontLimelight = new LimelightFront(LimelightConstants.FRONT_NAME, LimelightConstants.FRONT_POSE);
    private static final LimelightBack backLimelight = new LimelightBack(LimelightConstants.BACK_NAME, LimelightConstants.BACK_POSE);

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain(backLimelight, frontLimelight);
    private static final Arm arm = new Arm();
    private static final Wrist wrist = new Wrist(arm);
    private static final Elevator elevator = new Elevator();
    private static final ServoTurn servoturn = new ServoTurn();
    private static final Collector collector = new Collector();
    private static final LEDs leds = new LEDs(collector);
    private static final Lift lift = new Lift(elevator, wrist, arm);

    // Creates our controllers and deadzones
    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private static final XboxController copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER_PORT);
    private static final Joystick buttonPad = new Joystick(ControllerConstants.BUTTON_PAD_CONTROLLER_PORT);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        new Trigger(collector::isStalling)
                .onTrue(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1)).alongWith(new InstantCommand(() -> copilot.setRumble(RumbleType.kBothRumble, 1))));
        new Trigger(collector::isStalling)
                .onFalse(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)).alongWith(new InstantCommand(() -> copilot.setRumble(RumbleType.kBothRumble, 0))));

        /* driver Controls */
        // RESETS
        new Trigger(() -> (driver.getBackButton() && driver.getStartButton())).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle));

        new Trigger(() -> driver.getPOV() == 0).onTrue(new InstantCommand(drivetrain::flipFR, drivetrain));
        new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(drivetrain::flipBL, drivetrain));
        new Trigger(() -> driver.getPOV() == 90).onTrue(new InstantCommand(drivetrain::flipBR, drivetrain));
        new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(drivetrain::flipFL, drivetrain));

        // GAME PIECE SET
        new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CONE)));
        new Trigger(driver::getLeftBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CUBE)));

        // new Trigger(driver::getYButton).onTrue(new InstantCommand(() -> drivetrain.moveToDesiredPose(autoFactory), drivetrain)).onFalse(new InstantCommand(drivetrain::stop, drivetrain));
        new Trigger(driver::getYButton).whileTrue(new SingleSubstationAlign(drivetrain, frontLimelight, collector));

        // SET DRIVE PODS TO 45
        new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.stop(), drivetrain));

        //AUTO ALIGN
        new Trigger(() -> buttonPad.getRawButton(1)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_1_POSE)));
        new Trigger(() -> buttonPad.getRawButton(2)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_2_POSE)));
        new Trigger(() -> buttonPad.getRawButton(3)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_3_POSE)));
        new Trigger(() -> buttonPad.getRawButton(4)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_4_POSE)));
        new Trigger(() -> buttonPad.getRawButton(5)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_5_POSE)));
        new Trigger(() -> buttonPad.getRawButton(6)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_6_POSE)));
        new Trigger(() -> buttonPad.getRawButton(7)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_7_POSE)));
        new Trigger(() -> buttonPad.getRawButton(8)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_8_POSE)));
        new Trigger(() -> buttonPad.getRawButton(9)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_9_POSE)));
        new Trigger(() -> buttonPad.getRawButton(12)).onTrue(new InstantCommand(() -> drivetrain.setDesiredPose(AutoAlignConstants.BluePoints.SLOT_10_POSE)));

        // SERVO
        new Trigger(driver::getStartButton).onTrue(new InstantCommand(servoturn::flickServo));

        // AutoAlign based on cone or cube
        new Trigger(driver::getBButton).whileTrue(new ConditionalCommand(new AprilTagLineUp(drivetrain, frontLimelight, collector), new RetroLineUp(drivetrain, frontLimelight, collector),
                () -> collector.getGamePiece() == GamePiece.CUBE));

        //AUTOBALANCE
        // new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain));

        /* copilot controls */
        //BIAS
        new Trigger(() -> copilot.getPOV() == 0).onTrue(new InstantCommand(() -> lift.adjustWrist(4), lift));
        new Trigger(() -> copilot.getPOV() == 180).onTrue(new InstantCommand(() -> lift.adjustWrist(-4), lift));
        new Trigger(() -> copilot.getPOV() == 90).onTrue(new InstantCommand(() -> lift.adjustArm(4), lift));
        new Trigger(() -> copilot.getPOV() == 270).onTrue(new InstantCommand(() -> lift.adjustArm(-4), lift));
        new Trigger(() -> copilot.getRightY() < -0.1).onTrue(new InstantCommand(() -> lift.adjustElevator(1)));
        new Trigger(() -> copilot.getRightY() > 0.1).onTrue(new InstantCommand(() -> lift.adjustElevator(-1)));

        //SETPOINTS
        new Trigger(copilot::getAButton).whileTrue(new Ground(lift, collector, () -> collector.getGamePiece()));
        new Trigger(copilot::getBButton).whileTrue(new Stow(lift));
        new Trigger(copilot::getYButton).whileTrue(new HighScore(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getXButton).whileTrue(new MidScore(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getLeftBumper).whileTrue(new SingleSubstationCollect(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getRightBumper).whileTrue(new DoubleSubstationCollect(lift));

        //FLICK
        new Trigger(() -> -copilot.getLeftY() > 0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(112))));
        new Trigger(() -> -copilot.getLeftY() < -0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(lift.getLastKnownGoodWristSetPoint()))));

        //BREAK
        new Trigger(copilot::getRightStickButton).onTrue(new InstantCommand(lift::breakLift));

        // COLLECTOR
        new Trigger(() -> (Math.abs(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()) > 0.1)).onTrue(new HoldPower(collector,
                () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), ControllerConstants.DEADBAND) - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), ControllerConstants.DEADBAND)));

        //DISABLE LIFT
        new Trigger(() -> copilot.getStartButton() && copilot.getBackButton())
                .onTrue(new SequentialCommandGroup(new InstantCommand(wrist::disableWrist), new InstantCommand(arm::disableArm), new InstantCommand(elevator::disableEle)));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        //Test paths 

        // Game paths
        //A paths
        autoFactory.makeTrajectory("A2[2]-M-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        autoFactory.makeTrajectory("A2[2]-M-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        autoFactory.makeTrajectory("A2[2]-M-HIGH-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        autoFactory.makeTrajectory("A2[2]-M-HIGH-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        autoFactory.makeTrajectory("A2[2.25]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        autoFactory.makeTrajectory("A2[2.5]-M-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(2.25, 1.25)); // Tested double ground
        autoFactory.makeTrajectory("A2[2.5]-M-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested double ground
        // autoFactory.makeTrajectory("A1[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Bit silly NOT tested
        // autoFactory.makeTrajectory("A2[3]-M-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds), new PathConstraints(3.25, 2.125)); // works 3 low
        // autoFactory.makeTrajectory("A2[3]-M-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds), new PathConstraints(3.25, 2.125)); //Not tested 3 low
        // autoFactory.makeTrajectory("A2[1]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested
        // autoFactory.makeTrajectory("A2[1]-M-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Should work
        // autoFactory.makeTrajectory("A2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested autoBalance better from other side
        //B paths
        autoFactory.makeTrajectory("B2[1]-C-LOW", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Works
        // autoFactory.makeTrajectory("B2[1]-M-C-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION), new PathConstraints(1, .5),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested
        autoFactory.makeTrajectory("B2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Needs more testing
        // autoFactory.makeTrajectory("B2[1]-C-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Works
        autoFactory.makeTrajectory("B2[1.5]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Works
        // autoFactory.makeTrajectory("B2[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested
        //C paths
        autoFactory.makeTrajectory("C2[1]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Should Work
        autoFactory.makeTrajectory("C2[1]-M-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested
        // autoFactory.makeTrajectory("C2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested autoBalance better from other side
        autoFactory.makeTrajectory("C2[2]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // Tested
        // autoFactory.makeTrajectory("C2[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested
        // autoFactory.makeTrajectory("C2[3]-M-Blue", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested
        // autoFactory.makeTrajectory("C2[3]-M-Red", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); // NOT tested

        Autonomous.register("ruh roh flick auto", new InstantCommand(servoturn::flickServo, servoturn));
    }

    @Override
    protected void configureDefaultCommands() {
        /*
         * Set up the default command for the drivetrain. The controls are for field-oriented driving: Left
         * stick Y axis -> forward and backwards movement Left stick X axis -> left and right movement Right
         * stick X axis -> rotation
         */
        // drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX() * Math.sqrt(2)), () -> joystickFilter.filter(driver.getLeftY() * Math.sqrt(2)),
        //         () -> -joystickFilter.filter(driver.getRightX())));
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND),
                () -> driver.getRightTriggerAxis() > 0.25, () -> driver.getLeftTriggerAxis() > 0.25));

        leds.setDefaultCommand(new SafeToScoreLED(leds, drivetrain, collector));

        // elevator.setDefaultCommand(new EleUpInCommunity(elevator, lift, drivetrain));

        // elevator.setDefaultCommand(
        // new ManualLift(() -> driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(),
        // () -> 0, () -> 0, arm, wrist, elevator));
        // collector.setDefaultCommand(new Collect(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), XboxControllerConstants.DEADBAND)
        //         - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), XboxControllerConstants.DEADBAND)));
    }

    @Override
    protected void configureSystemTests() {
        SystemTest.registerTest("fl drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getFrontLeftModule(), 0.25));

        SystemTest.registerTest("fr drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getFrontRightModule(), 0.25));

        SystemTest.registerTest("bl drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getBackLeftModule(), 0.25));

        SystemTest.registerTest("br drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getBackRightModule(), 0.25));
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected AutonomousCommandFactory getCommandFactory() {
        return autoFactory;
    }
}
