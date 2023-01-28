package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceBangBang extends CommandBase {
    private Drivetrain drivetrain;
    private double lastAngle;

    // add gyroscope
    //TODO: initialize tanktrain
    
    public AutoBalanceBangBang(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    
    @Override
    public void initialize() {
        lastAngle = drivetrain.getPitch2d().getDegrees();
    }

    
    @Override
    public void execute() {                     
        // TODO: tune the value of 0.05 to probably something a lot smaller
        if (drivetrain.getPitch2d().getDegrees() - lastAngle > 0.05 && drivetrain.getPitch2d().getDegrees() > DrivetrainConstants.OPTIMAL_PITCH) { 
            drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(0.1), 
                                                drivetrain.percentOutputToMetersPerSecond(0), 
                                                drivetrain.percentOutputToMetersPerSecond(0)));
        } else if (drivetrain.getPitch2d().getDegrees() - lastAngle > 0.05 && drivetrain.getPitch2d().getDegrees() < DrivetrainConstants.OPTIMAL_PITCH) {
            drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-0.1), 
                                                drivetrain.percentOutputToMetersPerSecond(0), 
                                                drivetrain.percentOutputToMetersPerSecond(0)));
        } 
        /*
        * this measures the change in angle and if the robot is actually on a slope
        * if it is, it will move forward until it is level and then it will move back
        */
        
        lastAngle = drivetrain.getPitch2d().getDegrees();
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
