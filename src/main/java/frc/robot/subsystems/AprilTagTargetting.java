package frc.robot.subsystems;

import java.io.IOException;
import java.net.InetAddress;
import java.net.URL;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DataLogManager;

public class AprilTagTargetting extends SubsystemBase{
    private final NetworkTable limelightTab = NetworkTableInstance.getDefault().getTable("limelight");
    private final   ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting Tab");
    
    //Rest API Values
    private double botPose;
    public double botPosX;
    public double botPosY;
    public double botHeading;

    //NetworkTable Values
    private final double[] botPoseBlue = limelightTab.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    private final GenericEntry botPoseXEntry = targetingTab.add("botPoseX", 0).getEntry();
    private final GenericEntry botPoseYEntry = targetingTab.add("botPoseY", 0).getEntry();
    private final GenericEntry botPoseHeadingEntry = targetingTab.add("botPoseHeading", 0).getEntry();
    
    

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
        updateDashboard();
    }
    
    /*
     * Updates shuffleboard values using NetworkTables absolute position values
     */
    private void updateDashboard() {

		// Vision Dashboard Data
	    botPoseXEntry.setDouble(botPoseBlue[0]);
        botPoseYEntry.setDouble(botPoseBlue[1]);
        botPoseHeadingEntry.setDouble(botPoseBlue[3]);
		
	}

    public double getBotPose(){
        return this.botPose;
    }

    /*
     * Estimates pose using Rest API (For logging purposes)
     */
    public void estimatePose() throws IOException
    {
        InetAddress address;
        address = InetAddress.getByName("10.8.62.101");
        
        while(!address.isReachable(10000));
            
        URL url = new URL("http://10.8.62.101:5807/results"); 
            
        Scanner sc = new Scanner(url.openStream());

        while (sc.hasNext()) {
            String line = sc.next();
            // System.out.println(line);

            try{
                String FID = line.substring(line.indexOf("fID"));
                FID = FID.substring(5, FID.indexOf(",", 5));

                String botPos = line.substring(line.indexOf("\"botpose\":["));
                botPos = botPos.substring(11, botPos.indexOf("]"));

                List<String> allBotVal = Arrays.asList(botPos.split(","));

                botPosX = Double.parseDouble(allBotVal.get(0));
                botPosY = Double.parseDouble(allBotVal.get(1));
                botHeading = Double.parseDouble(allBotVal.get(4));
            } catch(Exception e){
                System.out.println("No Data");
            }
        }
    }
}

