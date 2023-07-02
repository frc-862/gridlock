package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.swervelib.SwerveModule;

/**
 * Drives a single module at a given speed and angle
 */
public class DriveTest extends CommandBase {
    private final SwerveModule module;
    private final Drivetrain drivetrain;
    private double driveSpeed;
    private double driveAngle;

    /**
     * Drives a single module at a given speed and angle
     * 
     * @param drivetrain The drivetrain
     * @param module The module to drive
     * @param drivesSpeed The speed to drive at
     */
    public DriveTest(Drivetrain drivetrain, SwerveModule module, double drivesSpeed) {
        this.module = module;
        this.drivetrain = drivetrain;
        this.driveSpeed = drivesSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        module.set(driveSpeed, driveAngle);
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
