package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import com.fasterxml.jackson.annotation.JacksonInject.Value;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagTargetting extends SubsystemBase {


    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight-alice");

    private double horizAngleToTarget;
    private double[] botPose = limelightTab.getEntry("botpose").getDoubleArray(new double[6]);
    private double[] botPoseBlue = limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    private double[] botPoseRed = limelightTab.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    private double[] botPosCheck;

    private double timer = 0;
    private int pipelineNum = 0;

    //private int count = 0;

    //private boolean pipelineSwitched = false;

    public AprilTagTargetting() {
        limelightTab.getEntry("pipeline").setNumber(0);
        limelightTab.getEntry("check").setDouble(0);
        CommandScheduler.getInstance().registerSubsystem(this);
        initLogging();
        //switchPipelines();
    }
    
    @Override
    public void periodic() {

        botPose = limelightTab.getEntry("botpose").getDoubleArray(new double[6]);
        
        if(limelightTab.getEntry("tv").getDouble(0)==1){
            timer += 0.2;
        }
        
        botPosCheck = botPose;
        botPoseBlue = limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        botPoseRed = limelightTab.getEntry("botpose_wpired").getDoubleArray(new double[6]);


        if (botPose.length != 0) {
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TX", botPose[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TY", botPose[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose RZ", botPose[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TX", botPoseBlue[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TY", botPoseBlue[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue RZ", botPoseBlue[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red TX", botPoseRed[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vi  sion bot pose Red TY", botPoseRed[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red RZ", botPoseRed[5]);

            LightningShuffleboard.setDouble("limelight", "timer", timer);
            LightningShuffleboard.setDouble("limelight", "check", LightningShuffleboard.getDouble("limelight", "pipeline", 0));
        }
        //checkPeriodicCount();
        setPipeline();
        //limelightTab.getEntry("check").setDouble(limelightTab.getEntry("limelight").getNumber(0));
        //timer += 0.02;

    }

    public void initLogging() {
        if (botPose.length != 0) {
            DataLogger.addDataElement("Vision bot pose TX", () -> botPose[0]);
            DataLogger.addDataElement("Vision bot pose TY", () -> botPose[1]);
            DataLogger.addDataElement("Vision bot pose RZ", () -> botPose[5]);
            DataLogger.addDataElement("Vision bot pose Blue TX", () -> botPoseBlue[0]);
            DataLogger.addDataElement("Vision bot pose Blue TY", () -> botPoseBlue[1]);
            DataLogger.addDataElement("Vision bot pose Blue RZ", () -> botPoseBlue[5]);
            DataLogger.addDataElement("Vision bot pose Red TX", () -> botPoseRed[0]);
            DataLogger.addDataElement("Vision bot pose Red TY", () -> botPoseRed[1]);
            DataLogger.addDataElement("Vision bot pose Red RZ", () -> botPoseRed[5]);


            
        }
    }

    /*public void checkPeriodicCount(){
        count ++;
        System.out.println("The function is running.");
        if (count == 30){
            switchPipelines();
            System.out.println("fortnite");
            count = 0;
        }
    }*/

    public Pose2d getRobotPose() {

        return new Pose2d(new Translation2d(botPose[0], botPose[1]), Rotation2d.fromDegrees(botPose[5]));

    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz)
     * 
     * @return 3d bot pose
     */
    public double[] getBotPose() {
        return this.botPose;
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag
     * targetting The
     * second is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        limelightTab.getEntry("pipeline").setNumber(pipelineNum);
    }



    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside
     * of FOV, it
     * can't be)
     * 
     * @return Whether or not target offset is more than 29.8 degrees.
     */
    public boolean validTarget() {
        // 29.8d represents the LL2+'s max FOV, from center of camera to edge of frame.
        return Math.abs(this.horizAngleToTarget) < Constants.Vision.HORIZ_CAMERA_FOV;
    }

    /**
     * Gives us degree offset to adjust our rotation by.
     * 
     * @return degree offset from target.
     */
    public double autoAlign() {
        // Set pipeline num to 2, should be retroreflective tape pipeline.
        setPipelineNum(2);

        var hasTarget = limelightTab.getEntry("tv").getDouble(0);
        this.horizAngleToTarget = limelightTab.getEntry("tx").getDouble(0);

        boolean isOnTarget = isOnTarget(this.horizAngleToTarget);

        if (hasTarget == 1 && !isOnTarget && validTarget()) {
            return horizAngleToTarget;
        } else {
            return 0d;
        }
    }

    /**
     * Function to tell us whether or not we're on target (centered on vision tape)
     * 
     * @param expectedAngle Angle we're supposed to be at according to offset of
     *                      target supplied by
     *                      Limelight
     * @return Whether we're within acceptable tolerance of the target.
     */
    public boolean isOnTarget(double expectedAngle) {
        // Should put consideration into how accurate we want to be later on.
        return expectedAngle < Constants.Vision.HORIZ_DEGREE_TOLERANCE;
    }

    // Change "limelight" to "limelight-alice" when on gridlock
    /*private void switchPipelines(){
        if (pipelineSwitched == false){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            pipelineSwitched = true;
        }
        else if (pipelineSwitched == true){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            pipelineSwitched = false;

        }


    }*/

    private void setPipeline() {
        
        /*if (pipelineNum != (int) LightningShuffleboard.getDouble("limelight", "pipeline", 0)){
            timer = 0;
        }*/
        pipelineNum = (int) LightningShuffleboard.getDouble("limelight", "pipeline", 0);
       
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNum);
        //System.out.println(pipelineNum);
        
    }

}
