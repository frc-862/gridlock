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
import edu.wpi.first.wpilibj.DataLogManager;

public class AprilTagTargetting extends SubsystemBase{
    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight");
    private final ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting Tab");
    private double botPose;


    @Override
    public void periodic(){
        this.botPose = limelightTab.getEntry("botpose").getDouble(0);
        try {
            estimatePose();
        } catch (IOException e) {
            System.out.println(e);
        }
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

