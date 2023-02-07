package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.swervelib.SwerveModule;

public class DriveTest extends CommandBase {
    private final SwerveModule module;
    private final Drivetrain drivetrain;
    private double driveSpeed = 1;
    private double driveAngle = 0;

    public DriveTest(Drivetrain drivetrain, SwerveModule module, double drivesSpeed) {
        this.module = module;
        this.drivetrain = drivetrain;
        this.driveSpeed = drivesSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        module.set(drivetrain.velocityToDriveVolts(driveSpeed), driveAngle);
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
