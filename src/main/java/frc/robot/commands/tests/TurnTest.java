package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SystemTestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.logging.DataLogger;
import frc.thunder.swervelib.SwerveModule;

/**
 * Turns a single module a select number of times
 */
public class TurnTest extends CommandBase {
    private final SwerveModule module;
    private final Drivetrain drivetrain;
    private double driveSpeed = 0;
    private double driveAngle = 0;
    private boolean direction = false;
    private int revolutions = 0;

    /**
     * Turns a single module a select number of times
     * 
     * @param drivetrain The drivetrain
     * @param module The module to drive
     * @param direction The direction to turn
     */
    public TurnTest(Drivetrain drivetrain, SwerveModule module, boolean direction) {
        this.module = module;
        this.drivetrain = drivetrain;
        this.direction = direction;

        DataLogger.addDataElement("Current angle", () -> Math.toDegrees(module.getSteerAngle()));
        DataLogger.addDataElement("Target angle", () -> driveAngle);
        DataLogger.addDataElement("Bearing Difference", () -> getBearingDifference());

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Set power
        module.set(driveSpeed, Math.toRadians(driveAngle));
        // Checks if the module made it to the angle with in 3 degrees and has gone around less than
        // 2 times
        if ((Math.abs(getBearingDifference()) < SystemTestConstants.ANGLE_DEAD_ZONE) && (revolutions < SystemTestConstants.MAX_ROTATIONS_PER_DIRECTION)) {
            // If true clockwise, if false counterclockwise
            if (direction) {
                driveAngle += SystemTestConstants.DEGREES_INTERVAL_INCREASE;
                wrapAround();
            } else {
                driveAngle -= SystemTestConstants.DEGREES_INTERVAL_INCREASE;
                wrapAround();
            }
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

    // Calculate the diference between target angle and curent angle
    private double getBearingDifference() {
        double current = Math.toDegrees(module.getSteerAngle());
        return (((driveAngle - current + 540) % 360) - 180);
    }

    // Checks and keeps angle within 0 - 360
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
