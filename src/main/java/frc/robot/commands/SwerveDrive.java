package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Our servedrive command to control the drivetrain
 */
public class SwerveDrive extends CommandBase {
    // Decleare our drivetrain subsystems
    private final Drivetrain drivetrain;

    // Declare our translation supplies/joystick values in most cases
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier slowMode;
    private final BooleanSupplier robotCentric;

    private double leftX;
    private double leftY;
    private double rightX;

    /**
     * Creates a new SwerveDrive command.
     *
     * @param drivetrainSubsystem The drivetrain subsystem this command will run on
     * @param translationXSupplier The control input for the translation in the X direction
     * @param translationYSupplier The control input for the translation in the Y direction
     * @param rotationSupplier The control input for rotation
     */
    public SwerveDrive(Drivetrain drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier slowMode, BooleanSupplier robotCentric) {
        this.drivetrain = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.slowMode = slowMode;
        this.robotCentric = robotCentric;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        // Get values from double suppliers

        if(slowMode.getAsBoolean()) {
            leftX = m_translationXSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_TRANSLATIONAL_MULT;
            leftY = m_translationYSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_TRANSLATIONAL_MULT;
            rightX = m_rotationSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_ROTATIONAL_MULT;
        } else {
            leftX = m_translationXSupplier.getAsDouble();
            leftY = m_translationYSupplier.getAsDouble();
            rightX = m_rotationSupplier.getAsDouble();
        }

        Rotation2d theta = Rotation2d.fromRadians(Math.atan2(leftY, leftX));
        double mag = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

        double xOut = Math.pow(mag, 3) * theta.getCos();

        double yOut = Math.pow(mag, 3) * theta.getSin();

        double zOut = Math.pow(rightX, 3);

        if(!robotCentric.getAsBoolean()) {
            drivetrain.drive(
                // Supply chassie speeds from the translation suppliers using feild relative control
                // TODO: x and y fliped
                ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.percentOutputToMetersPerSecond(-xOut), drivetrain.percentOutputToMetersPerSecond(yOut),
                        drivetrain.percentOutputToRadiansPerSecond(zOut), drivetrain.getYaw2d()));
        } else {
            // create robot relative speeds
            drivetrain.drive(
                    new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-yOut), drivetrain.percentOutputToMetersPerSecond(-xOut),
                            drivetrain.percentOutputToRadiansPerSecond(zOut)));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stops drivetrain
        drivetrain.stop();
    }
}
