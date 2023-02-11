package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagTargetting extends SubsystemBase {

    //Change "limelight-alice" to whatever the name of the limelight you are currently using is
    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight-alice");

    // Gets the horizontal angle to the target from the limelight
    private double horizAngleToTarget;

    // Gets Bot Pose from the limelight (x,y,z,rx,ry,rz)
    private double[] botPose = limelightTab.getEntry("botpose").getDoubleArray(new double[6]);
    private double[] botPoseBlue =
            limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    private double[] botPoseRed =
            limelightTab.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    private double hasVision = 
            limelightTab.getEntry("tv").getDouble(0);

    private double horizontalOffset = limelightTab.getEntry("tx").getDouble(0);
    private double verticalOffset = limelightTab.getEntry("ty").getDouble(0);
    private double targetVertical = limelightTab.getEntry("ta").getDouble(0);
    private double targetVisible = limelightTab.getEntry("ta").getDouble(0);

    public AprilTagTargetting() {
        // Inits logging for vision
        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        // Gets Bot Pose from the limelight periodic (x,y,z,rx,ry,rz)
        botPose = limelightTab.getEntry("botpose").getDoubleArray(new double[6]);
        botPoseBlue = limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        botPoseRed = limelightTab.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        hasVision = limelightTab.getEntry("tv").getDouble(0);

        if (botPose.length != 0) {
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TX", botPose[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TY", botPose[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose RZ", botPose[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TX", botPoseBlue[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TY", botPoseBlue[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue RZ", botPoseBlue[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red TX", botPoseRed[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red TY", botPoseRed[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red RZ", botPoseRed[5]);

        }
        horizontalOffset = limelightTab.getEntry("tx").getDouble(0);
        verticalOffset = limelightTab.getEntry("ty").getDouble(0);
        targetVertical = limelightTab.getEntry("tvert").getDouble(0);
        targetVisible = limelightTab.getEntry("tv").getDouble(0);
        if(targetVisible == 1){
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Horizontal Offset", horizontalOffset);
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Vertical Offset", verticalOffset);
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Target Area", targetVertical);
        }
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

    // Method to update the dashboard with current vision values
    public void updateDashboard() {
        if (botPose.length != 0) {
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TX", botPose[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose TY", botPose[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose RZ", botPose[5]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TX",
                    botPoseBlue[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue TY",
                    botPoseBlue[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Blue RZ",
                    botPoseBlue[5]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TX", botPoseRed[0]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red TY", botPoseRed[1]);
            LightningShuffleboard.setDouble("Autonomous", "Vision bot pose Red RZ", botPoseRed[5]);
        }

    }

    // Returns the robot pose as a Pose2d from vision data
    public Pose2d getRobotPose() {
        if (hasVision == 1){
            return new Pose2d(new Translation2d(botPose[0], botPose[1]),
                Rotation2d.fromDegrees(botPose[5]));
        }
        else {
            return null;
        }
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
     * Sets the pipeline we're using on the limelight. The first is for april tag targetting The
     * second is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        limelightTab.getEntry("pipeline").setNumber(pipelineNum);
    }

    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside of FOV, it
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
    public double retroAngleOffset() {
        // Set pipeline num to 2, should be retroreflective tape pipeline.
        setPipelineNum(2);

        // Getting the valid target and horizAngleToTarget entry from the limelight
        var hasTarget = limelightTab.getEntry("tv").getDouble(0);
        this.horizAngleToTarget = limelightTab.getEntry("tx").getDouble(0);

        // A check for if we're on target
        boolean isOnTarget = isOnTarget(horizAngleToTarget);

        // If were on target
        if (hasTarget == 1 && !isOnTarget && validTarget()) {
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
        return Math.abs(expectedAngle) < Constants.Vision.HORIZ_DEGREE_TOLERANCE;
    }
}
