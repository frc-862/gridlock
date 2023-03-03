package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import java.util.HashMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Collector.GamePiece;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.ServoTurn;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoAlign;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Collect;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.HoldPower;
import frc.robot.commands.Lift.DoubleSubstationCollect;
import frc.robot.commands.Lift.Ground;
import frc.robot.commands.Lift.HighScore;
import frc.robot.commands.Lift.MidScore;
import frc.robot.commands.Lift.ReverseDoubleSubstationCollect;
import frc.robot.commands.Lift.Stow;
import frc.robot.commands.tests.DriveTrainSystemTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.thunder.LightningContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.LimelightConstants;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private static final LimelightFront frontLimelight = new LimelightFront(LimelightConstants.FRONT_NAME, LimelightConstants.FRONT_POSE);
    private static final LimelightBack backLimelight = new LimelightBack(LimelightConstants.BACK_NAME, LimelightConstants.BACK_POSE);

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain(frontLimelight,backLimelight);
    private static final Arm arm = new Arm();
    private static final Wrist wrist = new Wrist(arm);
    private static final Elevator elevator = new Elevator();
    private static final ServoTurn servoturn = new ServoTurn();
    private static final Lift lift = new Lift(elevator, wrist, arm);
    private static final Collector collector = new Collector();

    // Creates our controllers and deadzones
    private static final XboxController driver = new XboxController(XboxControllerConstants.DRIVER_CONTROLLER_PORT);
    private static final XboxController copilot = new XboxController(XboxControllerConstants.COPILOT_CONTROLLER_PORT);
    private static final JoystickFilter joystickFilter = new JoystickFilter(XboxControllerConstants.DEADBAND, XboxControllerConstants.MIN_POWER, XboxControllerConstants.MAX_POWER, Mode.CUBED);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        //Driver Controls
        // Back button to reset field centeric driving to current heading of the robot
        new Trigger(driver::getBackButton).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle));

        // new Trigger(copilot::getAButton).onTrue(new InstantCommand(() -> elevator.setExtension(4)));
        // new Trigger(copilot::getBButton).onTrue(new InstantCommand(() -> arm.setAngle(Rotation2d.fromDegrees(-75))));
        // new Trigger(copilot::getXButton).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(-90))));

        // new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain));

        new Trigger(driver::getYButton).whileTrue(new AutoAlign(drivetrain, frontLimelight));


        // copilot controls 
        new Trigger(() -> (copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()) > 0.1).whileTrue(new Collect(collector, () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));
        new Trigger(() -> copilot.getPOV() == 0).whileTrue(new RunCommand(() -> lift.adjustWrist(1), lift));
        new Trigger(() -> copilot.getPOV() == 180).whileTrue(new RunCommand(() -> lift.adjustWrist(-1), lift));
        new Trigger(() -> copilot.getPOV() == 90).whileTrue(new RunCommand(() -> lift.adjustArm(1), lift));
        new Trigger(() -> copilot.getPOV() == 270).whileTrue(new RunCommand(() -> lift.adjustArm(-1), lift));

        // new Trigger(copilot::getAButton)
        //         .whileTrue((new ParallelCommandGroup(new InstantCommand(() -> arm.setAngle(Rotation2d.fromDegrees(-45))), new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(0))), new InstantCommand(() -> elevator.setExtension(6)))));
        // new Trigger(copilot::getYButton)
        //         .whileTrue(new ParallelCommandGroup(new InstantCommand(() -> arm.setAngle(Rotation2d.fromDegrees(-70))), new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(20))), new InstantCommand(() -> elevator.setExtension(8))));
        // new Trigger(copilot::getXButton)
        //         .whileTrue(new ParallelCommandGroup(new InstantCommand(() -> arm.setAngle(Rotation2d.fromDegrees(0))), new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(-40))), new InstantCommand(() -> elevator.setExtension(4))));
        
        new Trigger(copilot::getAButton).whileTrue(new Ground(lift, collector, () -> GamePiece.CONE));
        new Trigger(copilot::getRightBumper).whileTrue(new Ground(lift, collector, () -> GamePiece.CUBE));

        new Trigger(copilot::getBButton).whileTrue(new Stow(lift)); 
        new Trigger(copilot::getYButton).whileTrue(new HighScore(lift, () -> collector.getGamePiece()));
        new Trigger(copilot::getXButton).whileTrue(new MidScore(lift, () -> collector.getGamePiece()));
        // new Trigger(() -> -copilot.getLeftY() > 0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(0))));
        new Trigger(() -> -copilot.getLeftY() > 0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(112))));
        new Trigger(() -> -copilot.getLeftY() < -0.25).onTrue(new InstantCommand(() -> wrist.setAngle(Rotation2d.fromDegrees(0))));
        // new Trigger(copilot::getLeftBumper).whileTrue(new DoubleSubstationCollect(lift));

        new Trigger(copilot::getRightStickButton).onTrue(new InstantCommand(lift::targetOverride));

    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        //Test paths 
        // autoFactory.makeTrajectory("Meter", new HashMap<>(), new PathConstraints(1, 0.25));
        // autoFactory.makeTrajectory("Straight", new HashMap<>(), new PathConstraints(11, 3));
        // autoFactory.makeTrajectory("StraightButRotate", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, 1));
        // autoFactory.makeTrajectory("7Meter", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("StraightAndBack", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("StraightAndBackCurve", new HashMap<>(), new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        
        // Game paths
        autoFactory.makeTrajectory("Path1StartB", Maps.getPathMap1Piece(drivetrain, servoturn), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("Path2StartB", Maps.getPathMap1Piece(drivetrain, servoturn), 
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY,AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path3StartA", Maps.getPathMap1Piece(drivetrain, servoturn), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); 
        autoFactory.makeTrajectory("Path4StartA", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path4StartACharge", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path5StartC", Maps.getPathMap1Piece(drivetrain, servoturn), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION)); 
        autoFactory.makeTrajectory("Path6StartC", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path6StartCCharge", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path7StartA", Maps.getPathMap3Piece(drivetrain, servoturn, lift, collector), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("Path8StartC", Maps.getPathMap3Piece(drivetrain, servoturn, lift, collector), 
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("Path9StartB", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path9StartBCharge", Maps.getPathMap2Piece(drivetrain, servoturn, lift, collector),
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path10StartCCHarge", Maps.getPathMap1Piece(drivetrain, servoturn), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Path11StartACHarge", Maps.getPathMap1Piece(drivetrain, servoturn), 
                new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
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
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(-driver.getLeftX(), XboxControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driver.getLeftY(), XboxControllerConstants.DEADBAND), () -> -MathUtil.applyDeadband(driver.getRightX(), XboxControllerConstants.DEADBAND)));

        // elevator.setDefaultCommand(
        // new ManualLift(() -> driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(),
        // () -> 0, () -> 0, arm, wrist, elevator));
        collector.setDefaultCommand(new HoldPower(collector, () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));
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
