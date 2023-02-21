package frc.robot.commands;

    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.wpilibj2.command.CommandBase;
    import frc.robot.subsystems.Drivetrain;
    import frc.robot.subsystems.Vision;
import frc.thunder.shuffleboard.LightningShuffleboard;

    public class AutoAlign extends CommandBase {
        private Drivetrain drivetrain;
        private Vision vision;

    /** Creates a new AutoAlign. */ 
    public AutoAlign(Drivetrain drivetrain, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LightningShuffleboard.setDouble("AutoAlign", "Degree offset from tar", vision.autoAlign());
        LightningShuffleboard.setDouble("AutoAlign", "Distance from tar", vision.calculateRetroReflectiveDistance());

        //Gets distance to travel along the y axis
        double distance = vision.calculateRetroReflectiveDistance() * Math.sin(vision.autoAlign());
        LightningShuffleboard.setDouble("AutoAlign", "Distance on y", distance);
        // TODO: Tune this value IT is the scalar for speed 
        distance *= 0.25; 
        // THis is kinda dumb

        drivetrain.setChassisSpeeds(new ChassisSpeeds(0d, distance, 0d));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}