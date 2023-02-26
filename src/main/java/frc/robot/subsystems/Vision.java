package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.shuffleboard.LightningShuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The vision subsystem
 */
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

    private double llForward = 0.1524;
    private double llRight = 0.14224;

    public Vision() {
        // Sets the appropriate camera position
        setCameraPose();

        // Registers this as a proper Subsystem
        CommandScheduler.getInstance().registerSubsystem(this);

        setDriverCam();
    }

    // Method to update shuffleboard with vision data
    private void updateShuffleboard() {
            LightningShuffleboard.setDouble("Vision", "Vision bot pose TX", getBotPose()[0]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose TY", getBotPose()[1]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose RZ", getBotPose()[5]);

            LightningShuffleboard.setDouble("Vision", "Vision bot pose Blue TX", getBotPoseBlue()[0]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Blue TY", getBotPoseBlue()[1]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Blue RZ", getBotPoseBlue()[5]);

            LightningShuffleboard.setDouble("Vision", "Vision bot pose Red TX", getBotPoseRed()[0]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Red TY", getBotPoseRed()[1]);
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Red RZ", getBotPoseRed()[5]);

            LightningShuffleboard.setDoubleArray("Vision", "Vision robot bot pose", () -> new double[] {getRobotPose().getX(), getRobotPose().getY(), getRobotPose().getRotation().getDegrees()});
            LightningShuffleboard.setDoubleArray("Vision", "Vision robot bot pose blue",
                    () -> new double[] {getRobotPoseBlue().getX(), getRobotPoseBlue().getY(), getRobotPoseBlue().getRotation().getDegrees()});
            LightningShuffleboard.setDoubleArray("Vision", "Vision robot bot pose red",
                    () -> new double[] {getRobotPoseRed().getX(), getRobotPoseRed().getY(), getRobotPoseRed().getRotation().getDegrees()});

            LightningShuffleboard.setDouble("Vision", "RR Tape Horizontal Offset", getHorizontalOffset());
            LightningShuffleboard.setDouble("Vision", "RR Tape Vertical Offset", getVerticalOffset());
            LightningShuffleboard.setDouble("Vision", "RR Tape Target Area", getTargetArea());

            LightningShuffleboard.setDouble("Vision", "Vision latency pipeline", getLatencyPipline());
            LightningShuffleboard.setDouble("Vision", "Vision latency capture", getLatencyCapture());
            LightningShuffleboard.setDouble("Vision", "Vision bot pose latency", getLatencyBotPose());
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Blue latency", getLatencyBotPoseBlue());
            LightningShuffleboard.setDouble("Vision", "Vision bot pose Red latency", getLatencyBotPoseRed());

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
            
            LightningShuffleboard.setBool("Vision", "Vision", getHasVision());
    }

    /**
     * Gets the robot pose relative to the april tag
     * 
     * @return Pose2d of the robot
     */
    public Pose2d getRobotPose() {
        if (getHasVision()) {
            return new Pose2d(new Translation2d(getBotPose()[0] - llForward, getBotPose()[1] - llRight), Rotation2d.fromDegrees(getBotPose()[5]));
        } else {
            return new Pose2d();
        }
    }

    /**
     * Gets the robot pose relative to the blue side of the field
     * 
     * @return Pose2d of the robot
     */
    public Pose2d getRobotPoseBlue() {
        if (getHasVision()) {
            return new Pose2d(new Translation2d(getBotPoseBlue()[0] - llForward, getBotPoseBlue()[1] - llRight), Rotation2d.fromDegrees(getBotPoseBlue()[5]));
        } else {
            return new Pose2d();
        }
    }

    /**
     * Gets the robot pose relative to the red side of the field
     * 
     * @return Pose2d of the robot
     */
    public Pose2d getRobotPoseRed() {
        if (getHasVision()) {
            return new Pose2d(new Translation2d(getBotPoseRed()[0] - llForward, getBotPoseRed()[1] - llRight), Rotation2d.fromDegrees(getBotPoseRed()[5]));
        } else {
            return new Pose2d();
        }
    }

    /**
     * Gets the horizontal offset of the target from the center of the screen
     * 
     * @return double base on the horizontal FOV of the limelight
     */
    public double getHorizontalOffset() {
        if (getHasVision()) {
            horizontalOffset = LimelightHelpers.getTX(limelightName);
            return horizontalOffset;
        } else {
            return 0;
        }
    }

    /**
     * Gets the vertical offset of the target from the center of the screen
     * 
     * @return double base on the vertical FOV of the limelight
     */
    public double getVerticalOffset() {
        if (getHasVision()) {
            verticalOffset = LimelightHelpers.getTY(limelightName);
            return verticalOffset;
        } else {
            return 0;
        }
    }

    /**
     * Gets the area on the screen that the target takes up as a percentage
     * 
     * @return double between 0 and 100 percent of the image
     */
    public double getTargetArea() {
        if (getHasVision()) {
            targetVertical = LimelightHelpers.getTA(limelightName);
            return targetVertical;
        } else {
            return 0;
        }
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz) relative to the limelight
     * 
     * @return 7 element array of doubles
     */
    private double[] getBotPose() {
        if (getHasVision()) {
            botPose = LimelightHelpers.getBotPose(limelightName);
            return botPose;
        } else {
            return new double[] {0, 0, 0, 0, 0, 0, 0};
        }
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz) relative to red side of the field
     * 
     * @return 7 element array of doubles
     */
    private double[] getBotPoseRed() {
        if (getHasVision()) {
            botPoseRed = LimelightHelpers.getBotPose_wpiRed(limelightName);
            return botPoseRed;
        } else {
            return new double[] {0, 0, 0, 0, 0, 0, 0};
        }
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz) relative to blue side of the field
     * 
     * @return 7 element array of doubles
     */
    private double[] getBotPoseBlue() {
        if (getHasVision()) {
            botPoseBlue = LimelightHelpers.getBotPose_wpiBlue(limelightName);
            return botPoseBlue;
        } else {
            return new double[] {0, 0, 0, 0, 0, 0, 0};
        }
    }

    /**
     * Checks if we have a valid vision target
     * 
     * @return true if we have a valid vision target
     */
    public boolean getHasVision() {
        hasVision = LimelightHelpers.getTV(limelightName);
        return hasVision;
    }

    /**
     * Gets the latency of the pipeline
     * 
     * @return latency of pipeline
     */
    public double getLatencyPipline() {
        if (getHasVision()) {
            latencyPipline = LimelightHelpers.getLatency_Pipeline(limelightName);
            return latencyPipline;
        } else {
            return 0;
        }
    }

    /**
     * Gets the latency of the capture
     * 
     * @return latency of capture
     */
    public double getLatencyCapture() {
        if (getHasVision()) {
            latencyCapture = LimelightHelpers.getLatency_Capture(limelightName);
            return latencyCapture;
        } else {
            return 0;
        }
    }

    /**
     * Gets the latency of the bot pose
     * 
     * @return latency of bot pose
     */
    public double getLatencyBotPose() {
        if (getHasVision()) {
            var pose = LimelightHelpers.getBotPose(limelightName);
            if (pose.length != 0) {
                botPoseTotalLatency = LimelightHelpers.getBotPose(limelightName)[6];
                return botPoseTotalLatency;
            }
            return 0;

        } else {
            return 0;
        }
    }

    /**
     * Gets the latency of the blue bot pose
     * 
     * @return latency of blue bot pose
     */
    public double getLatencyBotPoseBlue() {
        if (getHasVision()) {
            var pose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
            if (pose.length != 0) {
                botPoseBlueTotalLatency = LimelightHelpers.getBotPose_wpiBlue(limelightName)[6];
                return botPoseBlueTotalLatency;
            }
            return 0;
        } else {
            return 0;
        }
    }

    /**
     * Gets the latency of the red bot pose
     * 
     * @return latency of red bot pose
     */
    public double getLatencyBotPoseRed() {
        if (getHasVision()) {
            var pose = LimelightHelpers.getBotPose_wpiRed(limelightName);
            if (pose.length != 0) {
                botPoseRedTotalLatency = LimelightHelpers.getBotPose_wpiRed(limelightName)[6];
                return botPoseRedTotalLatency;
            }
            return 0;
        } else {
            return 0;
        }
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag targetting The second
     * is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineNum);
    }

    public double getPipelineNum() {
        if (getHasVision()) {
            curPipeline = LimelightHelpers.getCurrentPipelineIndex(limelightName);
            return this.curPipeline;
        } else {
            return 0;
        }
    }

    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside of FOV, it can't
     * be)
     * 
     * @return Whether or not target offset is more than 29.8 degrees.
     */
    public boolean validTarget() {
        // 29.8d represents the LL2+'s max FOV, from center of camera to edge of frame.
        return Math.abs(getHorizontalOffset()) < Constants.VisionConstants.HORIZ_CAMERA_FOV;
    }

    /**
     * Gives us degree offset to adjust our rotation by.
     * 
     * @return degree offset from target.
     */
    public double autoAlign() {
        // Set pipeline num to 2, should be retroreflective tape pipeline.
        setPipelineNum(2);

        // Checks our current angle on the target
        if (getHasVision() && !isOnTarget(getHorizontalOffset()) && validTarget()) {
            return horizAngleToTarget;
        } else {
            return 0d;
        }
    }

    /**
     * Function to tell us whether or not we're on target (centered on vision tape)
     * 
     * @param expectedAngle Angle we're supposed to be at according to offset of target supplied by
     *        Limelight
     * @return Whether we're within acceptable tolerance of the target.
     */
    public boolean isOnTarget(double expectedAngle) {
        // Should put consideration into how accurate we want to be later on.
        return expectedAngle < Constants.VisionConstants.HORIZ_DEGREE_TOLERANCE;
    }

    private void setCameraPose() {
        if (limelightName == "limelight-front") {
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, 0.1524, 0.14224, 0.9398, 0, 0, 0);
        } else {
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, 0.1524, -0.14224, 0.9398, 0, 0, 0);
        }
    }

    // Sets the pipeline based on what is put in shuffleboard
    private void setPipeline() {
        // Gets the current shuffleboard value for the Pipeline entry
        pipelineNum = (int) LimelightHelpers.getLimelightNTDouble("limelight", "pipeline");

        // Updates the Limelight pipeline accordingly if pipelineNum is different than the current pipeline
        if (pipelineNum != getPipelineNum()) {
            LimelightHelpers.setLimelightNTDouble("limelight", "pipeline", pipelineNum);
        }
    }

    @Override
    public void periodic() {
        setPipeline();

        // Starts logging and updates the shuffleboard
        updateShuffleboard();

    }

}
