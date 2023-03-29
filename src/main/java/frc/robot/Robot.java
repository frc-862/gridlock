package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.thunder.LightningRobot;
import frc.thunder.limelightlib.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LightningRobot {

    public Robot() {
        super(new RobotContainer());
    }

    @Override 
    public void teleopInit(){
        LimelightHelpers.setPipelineIndex(LimelightConstants.BACK_NAME, 0);
        LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_NAME, 0);
    }

}
