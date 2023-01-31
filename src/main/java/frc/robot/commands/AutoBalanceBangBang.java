package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceBangBang extends CommandBase {
    private Drivetrain drivetrain;
    private double lastAngle;
    private double delta;
    private double lastTime = 0;
    
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

        if(Timer.getFPGATimestamp() - lastTime > AutoBalanceConstants.THRESHOLD_TIME) {
            delta = Math.abs(drivetrain.getPitch2d().getDegrees() - lastAngle);
            lastTime = Timer.getFPGATimestamp();
            lastAngle = drivetrain.getPitch2d().getDegrees();
        }

        if (delta < AutoBalanceConstants.THRESHOLD_ANGLE && Math.abs(drivetrain.getPitch2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_PITCH) { 
            drivetrain.drive(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-Math.signum(drivetrain.getPitch2d().getDegrees()) * 0.2), 
                                                          drivetrain.percentOutputToMetersPerSecond(0), 
                                                          drivetrain.percentOutputToMetersPerSecond(0)));
        } else {
            drivetrain.drive(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(0),
                drivetrain.percentOutputToMetersPerSecond(0), 
                drivetrain.percentOutputToMetersPerSecond(0)));
        } 
        /*
        * this measures the change in angle and if the robot is actually on a slope
        * if it is, it will move forward until it is level and then it will move back
        */
        
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
