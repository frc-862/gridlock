package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceBangBang extends CommandBase {
    private Drivetrain drivetrain;
    // add gyroscope
    //TODO: initialize tanktrain
    
    public AutoBalanceBangBang(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    
    @Override
    public void initialize() {
    }

    
    @Override
    public void execute() {
        double previousAngle = drivetrain.getPitch().getDegrees();
                                                
        // TODO: tune the value of 0.05 to probably something a lot smaller
        if (drivetrain.getPitch().getDegrees() - previousAngle > 0.05 && drivetrain.getPitch().getDegrees() > DrivetrainConstants.OPTIMAL_ROLL) { 
            drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(5), 
                                                drivetrain.percentOutputToMetersPerSecond(0), 
                                                drivetrain.percentOutputToMetersPerSecond(0)));
        } else if (drivetrain.getPitch().getDegrees() - previousAngle > 0.05 && drivetrain.getPitch().getDegrees() < DrivetrainConstants.OPTIMAL_ROLL) {
            drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-5), 
                                                drivetrain.percentOutputToMetersPerSecond(0), 
                                                drivetrain.percentOutputToMetersPerSecond(0)));
        } 

        // this measures the change in angle and if the robot is actually on a slope
        // if it is, it will move forward until it is level and then it will move back
        
        previousAngle = drivetrain.getPitch().getDegrees();
	}
                

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    
    @Override
    public boolean isFinished() {
        return false;
    }
}
