package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimelightFront;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector.GamePiece;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.ServoTurn;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoScore;
import frc.robot.commands.Collect;
import frc.robot.commands.EleUpInCommunity;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.HoldPower;
import frc.robot.commands.RetroLineUp;
import frc.robot.commands.AprilTagLineUp;
import frc.robot.commands.SafeToScoreLED;
import frc.robot.commands.SingleSubstationAlign;
import frc.robot.commands.Lift.ReverseDoubleSubStationCollect;
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
import frc.robot.Constants.LimelightConstants;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
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
        /* driver Controls */
        // RESETS
        new Trigger(() -> (driver.getBackButton() && driver.getStartButton())).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain)); // Resets Forward to be the direction the robot is facing

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle)); // REsyncs the NEOs relative encoder to the absolute encoders on the swerve modules

        // Flips modules 180 degrees to fix a module that is facing the wrong way on startup
        new Trigger(() -> driver.getPOV() == 0).onTrue(new InstantCommand(drivetrain::flipFR, drivetrain));
        new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(drivetrain::flipBL, drivetrain));
        new Trigger(() -> driver.getPOV() == 90).onTrue(new InstantCommand(drivetrain::flipBR, drivetrain));
        new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(drivetrain::flipFL, drivetrain));

        // GAME PIECE SET
        new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CONE)));
        new Trigger(driver::getLeftBumper).onTrue(new InstantCommand(() -> collector.setGamePiece(GamePiece.CUBE)));

        // new Trigger(driver::getYButton).onTrue(new InstantCommand(() -> drivetrain.moveToDesiredPose(autoFactory), drivetrain)).onFalse(new InstantCommand(drivetrain::stop, drivetrain));
        new Trigger(driver::getYButton).whileTrue(new SingleSubstationAlign(drivetrain, frontLimelight));

        // SET DRIVE PODS TO 45
        new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.stop(), drivetrain)); // Locks wheels to prevent sliding especially once balanced

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
        new Trigger(driver::getStartButton).onTrue(new InstantCommand(servoturn::flickServo)); // For testing Servo in the pits

        new Trigger(driver::getBButton).onTrue(new InstantCommand(() -> lift.switchVertical()));
        
        // AutoAlign based on cone or cube
        // new Trigger(driver::getBButton).whileTrue(new ConditionalCommand(new AprilTagLineUp(drivetrain, frontLimelight), new RetroLineUp(drivetrain, frontLimelight, collector),
        //         () -> collector.getGamePiece() == GamePiece.CUBE)); // WorKS but is slow

        //AUTOBALANCE
        // new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain)); // FOR TESTING

        /* copilot controls */
        //BIAS
        new Trigger(() -> copilot.getPOV() == 0).onTrue(new InstantCommand(() -> lift.adjustWrist(4), lift));
        new Trigger(() -> copilot.getPOV() == 180).onTrue(new InstantCommand(() -> lift.adjustWrist(-4), lift));
        new Trigger(() -> copilot.getPOV() == 90).onTrue(new InstantCommand(() -> lift.adjustArm(4), lift));
        new Trigger(() -> copilot.getPOV() == 270).onTrue(new InstantCommand(() -> lift.adjustArm(-4), lift));
        new Trigger(() -> copilot.getRightY() < -0.1).onTrue(new InstantCommand(() -> lift.adjustElevator(1)));
        new Trigger(() -> copilot.getRightY() > 0.1).onTrue(new InstantCommand(() -> lift.adjustElevator(-1)));

        //SETPOINTS
        new Trigger(copilot::getAButton).onTrue(new Ground(lift, collector, () -> collector.getGamePiece(), lift.getVertical()));
        new Trigger(copilot::getBButton).onTrue(new Stow(lift));
        new Trigger(copilot::getYButton).onTrue(new HighScore(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getXButton).onTrue(new MidScore(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getLeftBumper).onTrue(new SingleSubstationCollect(lift, () -> collector.getGamePiece()));
        // new Trigger(copilot::getRightBumper).onTrue(new DoubleSubstationCollect(lift)); 
        new Trigger(copilot::getRightBumper).onTrue(new ReverseDoubleSubStationCollect(lift)); // Disabled in teleop Used for testing
        
        //FLICK
        new Trigger(() -> -copilot.getLeftY() > 0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(150))));
        new Trigger(() -> -copilot.getLeftY() < -0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(lift.getLastKnownGoodWristSetPoint()))));

        //BREAK
        new Trigger(copilot::getRightStickButton).onTrue(new InstantCommand(lift::breakLift)); // Breaks out of current goal state and sets itself to onTarget so it can go to a new State

        //DISABLE LIFT
        new Trigger(() -> copilot.getStartButton() && copilot.getBackButton())
                .onTrue(new SequentialCommandGroup(new InstantCommand(wrist::disableWrist), new InstantCommand(arm::disableArm), new InstantCommand(elevator::disableEle)));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        //EXAMPLE
        // autoFactory.makeTrajectory("NAME", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));

        //A paths OPEN 
        autoFactory.makeTrajectory("A2[3]-M-BACK-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm), 
                new PathConstraints(3.5, 2));
        autoFactory.makeTrajectory("A2[3]-M-BACK-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm), 
                new PathConstraints(3.5, 2));
        //B paths MIDDLE
        autoFactory.makeTrajectory("B2[1]-M-C-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, .75));
        autoFactory.makeTrajectory("B2[1]-M-C-LOW", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        //C paths CABLE
        autoFactory.makeTrajectory("C2[2]-M-M-H-BLUE", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("C2[2]-M-M-H-RED", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("C2[3]-M-BACK", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds, arm), 
        //         new PathConstraints(3.5, 2));
        //ANYWHERE
        Autonomous.register("ruh roh flick auto", new InstantCommand(servoturn::flickServo, servoturn)); // Emergency Auton that doesn't drive
    }

    @Override
    protected void configureDefaultCommands() {
        /*
         * Set up the default command for the drivetrain. The controls are for field-oriented driving: Left
         * stick Y axis -> forward and backwards movement Left stick X axis -> left and right movement Right
         * stick X axis -> rotation
         */
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.DEADBAND),
                () -> driver.getRightTriggerAxis() > 0.25, () -> driver.getLeftTriggerAxis() > 0.25));

        leds.setDefaultCommand(new SafeToScoreLED(leds, drivetrain, collector)); // Changes LED color to RED when the arm will not hit when deploying 

        collector.setDefaultCommand(new HoldPower(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), ControllerConstants.DEADBAND) 
        - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), ControllerConstants.DEADBAND), driver, copilot, lift));

        // collector.setDefaultCommand(new Collect(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), ControllerConstants.DEADBAND) - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), ControllerConstants.DEADBAND)));

        // elevator.setDefaultCommand(new EleUpInCommunity(lift, drivetrain)); // Works problem is that Pos is not accurate enough
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


    /* Hello this is your favorite programing monster
     * 
     * I haunt your code and your dreams, you wont sleep at night while thinking about 
     * the code issues you've been having. 
     * 
     * have fun
     * 
     * bu bye >:)
     */




}
