// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SystemTestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.logging.DataLogger;
import frc.thunder.swervelib.SwerveModule;

public class TurnTest extends CommandBase {
    /** Creates a new FRDriveTest. */
    private final SwerveModule module;
    private final Drivetrain drivetrain;
    private double driveSpeed = 0;
    private double driveAngle = 0;
    private boolean direction = false;
    private int revolutions = 0;
    private double topAngle = 0;
    private double bottomAngle = 0;

    public TurnTest(Drivetrain drivetrain, SwerveModule module, boolean direction) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.module = module;
        this.drivetrain = drivetrain;
        this.direction = direction;
        DataLogger.addDataElement("Current angle", () -> Math.toDegrees(module.getSteerAngle()));
        DataLogger.addDataElement("Target angle", () -> driveAngle);
        DataLogger.addDataElement("Bearing Difference",
                () -> getBearingdifference(Math.toDegrees(module.getSteerAngle())));
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        bottomAngle = 0;
        topAngle = 0;

        // Set power
        module.set(drivetrain.velocityToDriveVolts(driveSpeed), Math.toRadians(driveAngle));
        // Checks if the module made it to the angle and has gone around less than 2 times
        if ((Math.abs(getBearingdifference(
                Math.toDegrees(module.getSteerAngle()))) < SystemTestConstants.ANGLE_DEAD_ZONE)
                && (revolutions < SystemTestConstants.MAX_ROTATIONS_PER_DIRECTION)) {
            if (direction) {// If true clockwise
                driveAngle += SystemTestConstants.DEGREES_INTERVAL_INCREASE;
                // Loops around
                wrapAround();
            } else if (!direction) {// If false counter Clockwise
                driveAngle -= SystemTestConstants.DEGREES_INTERVAL_INCREASE;
                // Loops around
                wrapAround();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double getBearingdifference(double cur) {
        // if (Math.signum(((tar - cur + 540) % 360) - 180) > 0) {
        return (((driveAngle - cur + 540) % 360) - 180);
    }

    private void wrapAround() {
        if ((driveAngle > 360)) {
            driveAngle -= 360;
            revolutions++;
        } else if ((driveAngle < 0)) {
            driveAngle += 360;
            revolutions++;
        }
    }
}
