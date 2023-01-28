package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;
    //TODO: initialize tanktrain
    private PIDController pid = new PIDController(Constants.AB_KP, Constants.AB_KI, Constants.AB_KD);
    
    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    
    @Override
    public void initialize() {
        SmartDashboard.putNumber("p gain", Constants.AB_KP);
    }

    
    @Override
    public void execute() {

        pid.setP(SmartDashboard.getNumber("P gain", Constants.AB_KP));

    if(Math.abs(drivetrain.getPitch2d().getDegrees()) > Constants.OPTIMAL_ROLL) { //maybe add check for theoretical color sensor?
      drivetrain.setChassisSpeeds(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(0),
                                                    drivetrain.percentOutputToMetersPerSecond(pid.calculate(drivetrain.getPitch2d().getDegrees(), 0)), 
                                                    drivetrain.percentOutputToMetersPerSecond(0)));

            

      SmartDashboard.putNumber("motor output", pid.calculate(drivetrain.getPitch2d().getDegrees(), 0));
    } else {
		drivetrain.stop();

        SmartDashboard.putNumber("motor output", 0);
	}
                
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
