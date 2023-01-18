package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagTargetting extends SubsystemBase{
    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight");
    private final ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting Tab");
     
    private double horizAngleToTarget = -1;
    private double botPose = -1;

    @Override
    public void periodic(){
        this.botPose = limelightTab.getEntry("botpose").getDouble(0);
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz) 
     * 
     * @return 3d bot pose
     */
    public double getBotPose(){
        return this.botPose;
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag targetting
     * The second is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    private void setPipelineNum(int pipelineNum){
        limelightTab.getEntry("pipeline").setNumber(pipelineNum);
    }

    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside of FOV, it can't be)
     * 
     * @return Whether or not target offset is more than 29.8 degrees.
     */
    private boolean validTarget(){
        // 29.8d represents the LL2+'s max FOV
        // TODO: Chuck 29.8 into constants file

        return Math.abs(this.horizAngleToTarget) < 29.8d;
    }

    /**
     * Gives us degree offset to adjust our rotation by. 
     * 
     * @return degree offset from target.
     */
    public double autoAlign(){
        setPipelineNum(2);
        
        var hasTarget = limelightTab.getEntry("tv").getDouble(0);
        this.horizAngleToTarget = limelightTab.getEntry("tx").getDouble(0);

        boolean isOnTarget = isOnTarget(this.horizAngleToTarget);

        if(hasTarget == 1 && !isOnTarget && validTarget()){
            return horizAngleToTarget;
        } else {
            return 0d;
        }
    }

    /**
     * Function to tell us whether or not we're on target (centered on vision tape)
     *  
     * @param expectedAngle Angle we're supposed to be at according to offset of target supplied by Limelight
     * @return Whether we're within acceptable tolerance of the target.
     */
    public boolean isOnTarget(double expectedAngle){
        // 4 is arbitrary angle in degrees that we should be within.
        // Should put consideration into how accurate we want to be later on.

        // TODO: Chuck deg tolerance in constants file later on
        double degTolerance = 4;
        return expectedAngle < degTolerance;
    }

}

