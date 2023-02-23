package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends SubsystemBase {

    // Change "limelight" to whatever the name of the limelight you are using
    // we should rename the limelight names to something consistent later
    private String limelightName = "limelight-front";

    // NetworkTableEntry

    // Setting values that we want to get later
    private double horizAngleToTarget;

    // Fiducial values
    private double[] botPose = LimelightHelpers.getBotPose(limelightName);
    private double[] botPoseBlue = LimelightHelpers.getBotPose_wpiBlue(limelightName);
    private double[] botPoseRed = LimelightHelpers.getBotPose_wpiRed(limelightName);

    // Both Fiducial and RetroReflective
    private boolean hasVision = LimelightHelpers.getTV(limelightName);

    // Read the README file in thunder's limelightlib for pipeline indexes
    private int pipelineNum = 1;
    private double curPipeline = LimelightHelpers.getCurrentPipelineIndex(limelightName);

    // RetroReflective values
    private double horizontalOffset = LimelightHelpers.getTX(limelightName);
    private double verticalOffset = LimelightHelpers.getTY(limelightName);
    private double targetVertical = LimelightHelpers.getTA(limelightName);

    // Getting limelight's latency
    private double latencyPipline = LimelightHelpers.getLatency_Pipeline(limelightName);
    private double latencyCapture = LimelightHelpers.getLatency_Capture(limelightName);

    private double botPoseTotalLatency;
    private double botPoseBlueTotalLatency;
    private double botPoseRedTotalLatency;

    //initial drivercam settings
    boolean driverCam = true;

    public Vision() {
        // Inits logging for vision
        initLogging();

        // Sets the appropriate camera position
        setCameraPose();

        // Registers this as a proper Subsystem
        CommandScheduler.getInstance().registerSubsystem(this);

        setDriverCam();
    }

    @Override
    public void periodic() {
        // if (botPose.length != 0) {
        // updateShuffleboard();

        // }

    }

    // Adds logging for vision so we can look at values when the robot is off and
    // check them
    public void initLogging() {
        // Checks if we have vision
        hasVision = LimelightHelpers.getTV(limelightName);

        // DataLogger.addDataElement("Has vision", () -> hasVision);

        DataLogger.addDataElement("Vision bot pose TX", () -> getBotPose()[0]);
        DataLogger.addDataElement("Vision bot pose TY", () -> getBotPose()[1]);
        DataLogger.addDataElement("Vision bot pose RZ", () -> getBotPose()[5]);

        DataLogger.addDataElement("Vision bot pose Blue TX", () -> getBotPoseBlue()[0]);
        DataLogger.addDataElement("Vision bot pose Blue TY", () -> getBotPoseBlue()[1]);
        DataLogger.addDataElement("Vision bot pose Blue RZ", () -> getBotPoseBlue()[5]);

        DataLogger.addDataElement("Vision bot pose Red TX", () -> getBotPoseRed()[0]);
        DataLogger.addDataElement("Vision bot pose Red TY", () -> getBotPoseRed()[1]);
        DataLogger.addDataElement("Vision bot pose Red RZ", () -> getBotPoseRed()[5]);

        DataLogger.addDataElement("Vision retro reflective TX", () -> getHorizontalOffset());
        DataLogger.addDataElement("Vision retro reflective TY", () -> getVerticalOffset());
        DataLogger.addDataElement("Vision retro reflective TA", () -> getTargetVertical());

        // DataLogger.addDataElement("Vision latency pipeline", () ->
        // getLatencyPipline());
        // DataLogger.addDataElement("Vision latency capture", () ->
        // getLatencyCapture());
        // DataLogger.addDataElement("Vision bot pose latency", () ->
        // getLatencyBotPose());
        // DataLogger.addDataElement("Vision bot pose Blue latency", () ->
        // getLatencyBotPoseBlue());
        // DataLogger.addDataElement("Vision bot pose Red latency", () ->
        // getLatencyBotPoseRed());

    }

    public void setDriverCam() {
        if (driverCam == true) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        } else if (driverCam == false) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
        }
    }

    public boolean getCamMode(){
        return driverCam;
    }

    // Returns the robot pose as a Pose2d from vision data
    public Pose2d getRobotPose() {
        if (hasVision) {
            return new Pose2d(new Translation2d(getBotPoseBlue()[0], getBotPoseBlue()[1]),
                    Rotation2d.fromDegrees(getBotPoseBlue()[5]));
        } else {
            return null;
        }
    }

    public double getHorizontalOffset() {
        horizontalOffset = LimelightHelpers.getTX(limelightName);
        return horizontalOffset;
    }

    public double getVerticalOffset() {
        verticalOffset = LimelightHelpers.getTX(limelightName);
        return verticalOffset;
    }

    public double getTargetVertical() {
        targetVertical = LimelightHelpers.getTX(limelightName);
        return targetVertical;
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz)
     * 
     * @return 3d bot pose
     */
    public double[] getBotPose() {
        botPose = LimelightHelpers.getBotPose(limelightName);
        if (botPose != null && botPose.length != 0) {
            return botPose;
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public double[] getBotPoseRed() {
        botPoseRed = LimelightHelpers.getBotPose_wpiRed(limelightName);
        if (botPoseRed != null && botPoseRed.length != 0) {
            return botPoseRed;
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public double[] getBotPoseBlue() {
        botPoseBlue = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        if (botPoseBlue != null && botPoseBlue.length != 0) {
            return botPoseBlue;
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public boolean getVision() {
        hasVision = LimelightHelpers.getTV(limelightName);
        return hasVision;
    }

    public double getLatencyPipline() {
        latencyPipline = LimelightHelpers.getLatency_Pipeline(limelightName);
        return latencyPipline;
    }

    public double getLatencyCapture() {
        latencyCapture = LimelightHelpers.getLatency_Capture(limelightName);
        return latencyCapture;
    }

    public double getLatencyBotPose() {
        var pose = LimelightHelpers.getBotPose(limelightName);
        if (pose.length != 0) {
            botPoseTotalLatency = LimelightHelpers.getBotPose(limelightName)[6];
            return botPoseTotalLatency;
        }
        return 0;
    }

    public double getLatencyBotPoseBlue() {
        var pose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        if (pose.length != 0) {
            botPoseBlueTotalLatency = LimelightHelpers.getBotPose_wpiBlue(limelightName)[6];
            return botPoseBlueTotalLatency;
        }
        return 0;
    }

    public double getLatencyBotPoseRed() {
        var pose = LimelightHelpers.getBotPose_wpiRed(limelightName);
        if (pose.length != 0) {
            botPoseRedTotalLatency = LimelightHelpers.getBotPose_wpiRed(limelightName)[6];
            return botPoseRedTotalLatency;
        }
        return 0;
    }

    private void updateShuffleboard() {
        if (pipelineNum == 0 || pipelineNum == 1) {
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TX", getBotPose()[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TY", getBotPose()[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose RZ", getBotPose()[5]);

            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TX",
                    getBotPoseBlue()[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TY",
                    getBotPoseBlue()[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue RZ",
                    getBotPoseBlue()[5]);

            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TX",
                    getBotPoseRed()[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TY",
                    getBotPoseRed()[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red RZ",
                    getBotPoseRed()[5]);
        } else if (pipelineNum == 2 || pipelineNum == 3) {
            LightningShuffleboard.setDouble("Autonomous", "RR Tape Horizontal Offset",
                    getHorizontalOffset());
            LightningShuffleboard.setDouble("Autonomous", "RR Tape Vertical Offset",
                    getVerticalOffset());
            LightningShuffleboard.setDouble("Autonomous", "RR Tape Target Area",
                    getTargetVertical());
        }
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag
     * targetting The
     * second is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineNum);
    }

    public double getPipelineNum() {
        curPipeline = LimelightHelpers.getCurrentPipelineIndex(limelightName);
        return this.curPipeline;
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
        return Math.abs(this.horizAngleToTarget) < Constants.VisionConstants.HORIZ_CAMERA_FOV;
    }

    /**
     * Gives us degree offset to adjust our rotation by.
     * 
     * @return degree offset from target.
     */
    public double autoAlign() {
        // Set pipeline num to 2, should be retroreflective tape pipeline.
        setPipelineNum(2);

        this.horizAngleToTarget = LimelightHelpers.getTX(limelightName);

        boolean isOnTarget = isOnTarget(this.horizAngleToTarget);

        // Checks if we have vision
        hasVision = LimelightHelpers.getTV(limelightName);

        // Checks our current angle on the target
        if (hasVision && !isOnTarget && validTarget()) {
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
        return expectedAngle < Constants.VisionConstants.HORIZ_DEGREE_TOLERANCE;
    }

    private void setCameraPose(){
        if (limelightName == "limelight-front"){
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, 0.1524, 0.14224, 0.9398, 0, 0, 0);
        } else {
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, 0.1524, -0.14224, 0.9398, 0, 0, 0);
        }
    }

    // Sets the pipeline based on what is put in shuffleboard
    private void setPipeline() {

        // Gets the current shuffleboard value for the Pipeline entry
        pipelineNum = (int) LimelightHelpers.getLimelightNTDouble("limelight", "pipeline");

        // Updates the Limelight pipeline accordingly if pipelineNum is different than
        // the current
        // pipeline
        if (pipelineNum != getPipelineNum()) {
            LimelightHelpers.setLimelightNTDouble("limelight", "pipeline", pipelineNum);
        }
    }

}
