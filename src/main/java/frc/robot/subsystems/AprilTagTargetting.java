package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagTargetting extends SubsystemBase {
    private final NetworkTable limelightTab =
            NetworkTableInstance.getDefault().getTable("limelight");
    private final ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting Tab");
    private double botPose;


    @Override
    public void periodic() {
        this.botPose = limelightTab.getEntry("botpose").getDouble(0);
    }

    public double getBotPose() {
        return this.botPose;
    }
}

