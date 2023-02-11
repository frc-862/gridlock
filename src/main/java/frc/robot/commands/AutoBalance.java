package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;
    private PIDController pid = new PIDController(AutoBalanceConstants.kP, AutoBalanceConstants.kI,
            AutoBalanceConstants.kD);

    private double lastPitch;
    private double lastRoll;
    private double pitchDelta;
    private double rollDelta;
    private double lastTime = 0;

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

        if (Timer.getFPGATimestamp() - lastTime > AutoBalanceConstants.THRESHOLD_TIME) {
            pitchDelta = Math.abs(drivetrain.getPitch2d().getDegrees() - lastPitch);
            rollDelta = Math.abs(drivetrain.getPitch2d().getDegrees() - lastRoll);
            lastTime = Timer.getFPGATimestamp();
            lastPitch = drivetrain.getPitch2d().getDegrees();
            lastRoll = drivetrain.getRoll2d().getDegrees();
        }


        // things commented out allow for roll conterol, but there are a couple issues that need to
        // be ironed out
        if ((/*
              * Math.abs(drivetrain.getRoll2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_ROLL ||
              */ Math
                .abs(drivetrain.getPitch2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_PITCH)
                && pitchDelta < AutoBalanceConstants.THRESHOLD_ANGLE) { // maybe add check for
                                                                        // theoretical color sensor?
            drivetrain.drive(new ChassisSpeeds(
                    drivetrain.percentOutputToMetersPerSecond(
                            pid.calculate(drivetrain.getPitch2d().getDegrees(), 0)),
                    drivetrain.percentOutputToMetersPerSecond(0), // -pid.calculate(drivetrain.getRoll2d().getDegrees(),
                                                                  // 0)),
                    drivetrain.percentOutputToMetersPerSecond(0)));
            SmartDashboard.putNumber("motor output",
                    pid.calculate(drivetrain.getPitch2d().getDegrees(), 0));
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
