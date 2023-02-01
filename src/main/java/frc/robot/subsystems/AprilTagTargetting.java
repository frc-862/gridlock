package frc.robot.subsystems;

import java.io.IOException;
import java.net.InetAddress;
import java.net.URL;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DataLogManager;

public class AprilTagTargetting extends SubsystemBase{
    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight");
    private final ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting Tab");
    
    private double botPose;
    private final double[] botPoseBlue = limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    private final NetworkTableEntry visionDistanceEntry = targetingTab.add("Vision Distance", 0).getEntry();
    //TODO: make this actually display a value
    

    //Constructor
    public AprilTagTargetting() {

    }

    @Override
    public void periodic(){
        this.botPose = limelightTab.getEntry("botpose").getDouble(0);
        try {
            estimatePose();
        } catch (IOException e) {
            System.out.println(e);
        }
        System.out.println(botPoseBlue[0]);     
        targetingTab.add("", 0).getEntry();
    }

    public double getBotPose(){
        return this.botPose;
    }

    public void estimatePose() throws IOException{
        InetAddress address;
        address = InetAddress.getByName("10.8.62.101");
        
        while(!address.isReachable(10000));
            
        URL url = new URL("http://10.8.62.101:5807/results"); 
            
        Scanner sc = new Scanner(url.openStream());

        while (sc.hasNext()) {
            String line = sc.next();
            System.out.println(line);
        }
    }
}

