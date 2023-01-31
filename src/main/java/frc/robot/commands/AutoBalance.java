package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;
    private PIDController pid = new PIDController(AutoBalanceConstants.kP, AutoBalanceConstants.kI, AutoBalanceConstants.kD);
    
    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    
    @Override
    public void initialize() {
        SmartDashboard.putNumber("p gain", AutoBalanceConstants.kP);
    }

    
    @Override
    public void execute() {
        pid.setP(SmartDashboard.getNumber("P gain", AutoBalanceConstants.kP));

        if(Math.abs(drivetrain.getPitch2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_ROLL) { //maybe add check for theoretical color sensor?
           drivetrain.drive(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(pid.calculate(drivetrain.getPitch2d().getDegrees(), 0)),
                                                drivetrain.percentOutputToMetersPerSecond(0), 
                                                drivetrain.percentOutputToMetersPerSecond(0)));
            SmartDashboard.putNumber("motor output", pid.calculate(drivetrain.getPitch2d().getDegrees(), 0));
        } else {
            // drivetrain.stop();
            drivetrain.drive(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(0),
                drivetrain.percentOutputToMetersPerSecond(0), 
                drivetrain.percentOutputToMetersPerSecond(0)));
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
