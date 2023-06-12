package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

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
    public SwerveDrive(Drivetrain drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier slowMode,
            BooleanSupplier robotCentric) {
        this.drivetrain = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.slowMode = () -> true;
        this.robotCentric = robotCentric;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // Get values from double suppliers

        if (slowMode.getAsBoolean()) { // Works switch back from Commented to 3 lines uncommented inside if statement
            // if(!drivetrain.isInLoadZone()) { // Was removed because the driver wanted a consistant speed in slow Mode  
            //     leftX = m_translationXSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_TRANSLATIONAL_MULT;
            //     leftY = m_translationYSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_TRANSLATIONAL_MULT;
            //     rightX = m_rotationSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_ROTATIONAL_MULT;
            // } else {
            //     leftY = m_translationYSupplier.getAsDouble() * 0.5;
            //     leftX = m_translationXSupplier.getAsDouble() * 0.5;
            //     rightX = m_rotationSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_ROTATIONAL_MULT;
            // }
            leftY = m_translationYSupplier.getAsDouble() * 0.5;
            leftX = m_translationXSupplier.getAsDouble() * 0.5;
            rightX = m_rotationSupplier.getAsDouble() * DrivetrainConstants.SLOW_MODE_ROTATIONAL_MULT;
        } else {
            leftX = m_translationXSupplier.getAsDouble();
            leftY = m_translationYSupplier.getAsDouble();
            rightX = m_rotationSupplier.getAsDouble() * 0.8;
        }

        // Get direction and magnitude of linear axes
        double linearMagnitude = Math.hypot(leftX, leftY);
        Rotation2d linearDirection = new Rotation2d(leftX, leftY);

        // Apply squaring
        linearMagnitude = Math.pow(linearMagnitude, 3);
        rightX = Math.pow(rightX, 3);

        // Calcaulate new linear components
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d())).getTranslation();

        if (!robotCentric.getAsBoolean()) { // Changes from field relative to robot relative to help with line up
            drivetrain.drive(
                    // Supply chassie speeds from the translation suppliers using feild relative control
                    // TODO: x and y fliped
                    ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.percentOutputToMetersPerSecond(-linearVelocity.getX()), drivetrain.percentOutputToMetersPerSecond(linearVelocity.getY()),
                            drivetrain.percentOutputToRadiansPerSecond(rightX), drivetrain.getYaw2d()));
        } else {
            // create robot relative speeds
            drivetrain.drive(new ChassisSpeeds(drivetrain.percentOutputToMetersPerSecond(-linearVelocity.getY()), drivetrain.percentOutputToMetersPerSecond(-linearVelocity.getX()), drivetrain.percentOutputToRadiansPerSecond(rightX)));
        }
    }

    @Override
    public void end(boolean interrupted) { // Stops drivetrain
        drivetrain.stop();
    }
}
