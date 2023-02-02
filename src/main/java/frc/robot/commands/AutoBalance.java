package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private double pitchAngle;
    private double rollAngle;
    private double totalAngle;
    private double offset = 0;
    private double finalAngle;
    private SwerveModuleState[] moduleStates;


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
            pitchAngle = drivetrain.getPitch2d().getDegrees();
            rollAngle = drivetrain.getRoll2d().getDegrees();
            totalAngle = Math.abs(pitchAngle) + Math.abs(rollAngle);
            if (pitchAngle >= 0){
                if (rollAngle >= 0){
                    offset = 315;
                } else {
                    offset = 45;
                }
            } else {
                if (rollAngle >= 0){
                    offset = 225;
                } else {
                    offset = 135;
                }
            }
            finalAngle = offset + 45*(pitchAngle/totalAngle - rollAngle/totalAngle);

            
            SmartDashboard.putNumber("final Angle", finalAngle);
        }



        // things commented out allow for roll conterol, but there are a couple issues that need to
        // be ironed out
        if ((
               Math.abs(drivetrain.getRoll2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_ROLL && rollDelta < AutoBalanceConstants.THRESHOLD_ROLL_ANGLE) ||
               (Math.abs(drivetrain.getPitch2d().getDegrees()) > AutoBalanceConstants.OPTIMAL_PITCH && pitchDelta < AutoBalanceConstants.THRESHOLD_PITCH_ANGLE)) { // maybe add check for
                                                                        // theoretical color sensor?
            //this code should make the robot autobalance using the final angle that is beng printed to smartdashboard
            // make sure someone reads over it though, because I'm not sure if I'm using the right drivetrain functions stuff - Alok
            //don't uncomment this until final angle has been checked to make sense
            /* 
            moduleStates = drivetrain.getStates();
            for (int m = 0; m < 4; m++){
                moduleStates[m].angle = Rotation2d.fromDegrees(finalAngle);
                moduleStates[m].speedMetersPerSecond = drivetrain.percentOutputToMetersPerSecond(pid.calculate(finalAngle));
            }
            */

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
