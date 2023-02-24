package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Limelight extends SubsystemBase {

    // Change "limelight" to whatever the name of the limelight you are using
    // we should rename the limelight names to something consistent later
    private String limelightName;

    public Limelight(String limelightName, Pose3d cameraPose) {
        this.limelightName = limelightName;
        // Inits logging for vision
        initLogging();

        // Sets the appropriate camera position
        setCameraPose(cameraPose);

        hasVision();

        // Registers this as a proper Subsystem
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // Adds logging for vision so we can look at values when the robot is off and
    // check them
    public void initLogging() {
        DataLogger.addDataElement("Has Vision", () -> hasVision() ? 1 : 0);
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
        DataLogger.addDataElement("Vision retro reflective TA", () -> getTargetArea());

        DataLogger.addDataElement("Vision latency pipeline", () -> getLatencyPipline());
        DataLogger.addDataElement("Vision latency capture", () -> getLatencyCapture());
        DataLogger.addDataElement("Vision bot pose latency", () -> getLatencyBotPose());
        DataLogger.addDataElement("Vision bot pose Blue latency", () -> getLatencyBotPoseBlue());
        DataLogger.addDataElement("Vision bot pose Red latency", () -> getLatencyBotPoseRed());
    }

    /**
     * Gets the robot pose relative to the april tag
     * 
     * @return Pose2d of the robot
     */
    public Pose2d getRobotPose() {
        if (hasVision()) {
            return new Pose2d(new Translation2d(getBotPoseBlue()[0], getBotPoseBlue()[1]),
                    Rotation2d.fromDegrees(getBotPoseBlue()[5]));
        } else {
            return null;
        }
    }

    /**
     * Gets the robot pose relative to the red side of the field
     * 
     * @return Pose2d of the robot
     */
    public Pose2d getRobotPoseRed() {
        if (hasVision()) {
            return new Pose2d(new Translation2d(getBotPoseRed()[0], getBotPoseRed()[1]),
                    Rotation2d.fromDegrees(getBotPoseRed()[5]));
        } else {
            return null;
        }
    }

    /**
     * Gets the horizontal offset of the target from the center of the screen
     * 
     * @return double base on the horizontal FOV of the limelight
     */
    public double getHorizontalOffset() {
        if (hasVision()) {
            return LimelightHelpers.getTX(limelightName);
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
        if (hasVision()) {
            return LimelightHelpers.getTX(limelightName);
        } else {
            return 0;
        }
    }

    /**
     * Gets the area of the target
     * 
     * @return double base on the area of the target
     */
    public double getTargetArea() {
        if (hasVision()) {
            return LimelightHelpers.getTX(limelightName);
        } else {
            return 0;
        }
    }

    public double getTargetVertical() {
        if (hasVision()) {
            return LimelightHelpers.getTX(limelightName);
        } else {
            return 0;
        }
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz) relative to the limelight
     * 
     * @return 7 element array of doubles
     */
    public double[] getBotPose() {
        if (hasVision()) {
            return LimelightHelpers.getBotPose(limelightName);
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public double[] getBotPoseRed() {
        if (hasVision()) {
            return LimelightHelpers.getBotPose_wpiRed(limelightName);
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public double[] getBotPoseBlue() {
        if (hasVision()) {
            return LimelightHelpers.getBotPose_wpiBlue(limelightName);
        } else {
            return new double[] { 0, 0, 0, 0, 0, 0, 0 };
        }
    }

    public boolean hasVision() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the latency of the pipeline
     * 
     * @return latency of pipeline
     */
    public double getLatencyPipline() {
        if (hasVision()) {
            return LimelightHelpers.getLatency_Pipeline(limelightName);
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
        if (hasVision()) {
            return LimelightHelpers.getLatency_Capture(limelightName);
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
        double[] pose = LimelightHelpers.getBotPose(limelightName);
        if (hasVision() && pose.length != 0) {
            return pose[6];
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
        double[] pose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        if (hasVision() && pose.length != 0) {
            return pose[6];
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
        double[] pose = LimelightHelpers.getBotPose_wpiRed(limelightName);
        if (hasVision() && pose.length != 0) {
            return pose[6];
        } else {
            return 0;
        }
    }

    public void updateShuffleboard() {
        double pipelineNum = getPipelineNum();
        if (hasVision()) {
            if (pipelineNum == 0 || pipelineNum == 1) {
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TX", getBotPose()[0]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TY", getBotPose()[1]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose RZ", getBotPose()[5]);

                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TX", getBotPoseBlue()[0]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TY", getBotPoseBlue()[1]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue RZ", getBotPoseBlue()[5]);

                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TX", getBotPoseRed()[0]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TY", getBotPoseRed()[1]);
                LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red RZ", getBotPoseRed()[5]);
            } else if (pipelineNum == 2 || pipelineNum == 3) {
                LightningShuffleboard.setDouble("Autonomous", "RR Tape Horizontal Offset", getHorizontalOffset());
                LightningShuffleboard.setDouble("Autonomous", "RR Tape Vertical Offset", getVerticalOffset());
                LightningShuffleboard.setDouble("Autonomous", "RR Tape Target Area", getTargetArea());
            }
        }
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag
     * targetting The second
     * is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineNum);
    }

    public int getPipelineNum() {
        if (hasVision()) {
            return (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        } else {
            return 0;
        }
    }

    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside
     * of FOV, it can't
     * be)
     * 
     * @return Whether or not target offset is more than 29.8 degrees.
     */
    public boolean validAngle(double angle) {
        // 29.8d represents the LL2+'s max FOV, from center of camera to edge of frame.
        return Math.abs(angle) < Constants.VisionConstants.HORIZ_CAMERA_FOV;
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

    private void setCameraPose(Pose3d pose) {
        if (hasVision()) {
            LimelightHelpers.setCameraPose_RobotSpace(limelightName, pose);
        }
    }

    // Sets the pipeline based on what is put in shuffleboard
    public void setPipeline() {
        if (hasVision()) {
            // Gets the current shuffleboard value for the Pipeline entry
            int pipelineNum = (int) LimelightHelpers.getLimelightNTDouble("limelight", "pipeline");

            // Updates the Limelight pipeline accordingly if pipelineNum is different than
            // the current
            // pipeline
            if (pipelineNum != getPipelineNum()) {
                LimelightHelpers.setLimelightNTDouble("limelight", "pipeline", pipelineNum);
            }
        }
    }

    @Override
    public void periodic() {
        updateShuffleboard();
        setPipeline();

    }

}
