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

    private final NetworkTableEntry horizontalOffset = limelightTab.getEntry("tx");
    private final NetworkTableEntry verticalOffset = limelightTab.getEntry("ty");
    private final NetworkTableEntry targetArea = limelightTab.getEntry("ta");

    double x = horizontalOffset.getDouble(0.0);
    double y = verticalOffset.getDouble(0.0);
    double area = targetArea.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
}
