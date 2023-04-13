package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.LimelightConstants;
import frc.thunder.LightningRobot;
import frc.thunder.auto.Autonomous;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.vision.VisionBase;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LightningRobot {

    // private double timer = 0;
    // private boolean timerStart = false;

    // private SendableChooser<String> chooser = new SendableChooser<>();
    // private int pos = -1;

    public Robot() {
        super(new RobotContainer());
    }

    // @Override
    // public void robotInit() {
    //     super.robotInit();

    //     ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    //     chooser.setDefaultOption("CLICK HERE, MIKE", "NONE");
    //     chooser.addOption("A", "A");
    //     chooser.addOption("B", "B");
    //     chooser.addOption("C", "C");
    //     tab.add("Auton type", chooser);
    // }

    // @Override
    // public void disabledPeriodic() {
    //     super.disabledPeriodic();
    //     switch (chooser.getSelected()) {
    //         case "A":
    //             if (!timerStart) {
    //                 timer = Timer.getFPGATimestamp();
    //             }

    //             timerStart = true;
    //             pos = 3;
    //             if (DriverStation.getAlliance() == Alliance.Blue) {
    //                 pos += 6;
    //             } else {
    //                 pos += 3;
    //             }

    //             LimelightHelpers.setPipelineIndex(LimelightConstants.BACK_NAME, pos);
    //             LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_NAME, pos);

    //             if (Timer.getFPGATimestamp() - timer > 1) {
    //                 VisionBase.enableVision();
    //             }

    //             break;

    //         case "B":
    //             if (!timerStart) {
    //                 timer = Timer.getFPGATimestamp();
    //             }

    //             timerStart = true;
    //             pos = 2;
    //             if (DriverStation.getAlliance() == Alliance.Blue) {
    //                 pos += 6;
    //             } else {
    //                 pos += 3;
    //             }
    //             LimelightHelpers.setPipelineIndex(LimelightConstants.BACK_NAME, pos);
    //             LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_NAME, pos);

    //             if (Timer.getFPGATimestamp() - timer > 1) {
    //                 VisionBase.enableVision();
    //             }
    //             break;

    //         case "C":
    //             if (!timerStart) {
    //                 timer = Timer.getFPGATimestamp();
    //             }

    //             timerStart = true;
    //             pos = 1;
    //             if (DriverStation.getAlliance() == Alliance.Blue) {
    //                 pos += 6;
    //             } else {
    //                 pos += 3;
    //             }
    //             LimelightHelpers.setPipelineIndex(LimelightConstants.BACK_NAME, pos);
    //             LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_NAME, pos);

    //             if (Timer.getFPGATimestamp() - timer > 1) {
    //                 VisionBase.enableVision();
    //             }

    //             break;

    //         case "NONE":
    //             VisionBase.disableVision();
    //             break;

    //     }
    // }

    // @Override
    // public void autonomousInit() {
    //     super.autonomousInit();
    // }

    // @Override
    // public void teleopInit() {
    //     super.teleopInit();
    //     LimelightHelpers.setPipelineIndex(LimelightConstants.BACK_NAME, 0);
    //     LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_NAME, 0);
    // }

}
