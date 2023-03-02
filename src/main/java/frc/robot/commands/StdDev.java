package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightBack;

public class StdDev extends CommandBase {

    private LimelightBack vision;

    private ArrayList<Double> x;
    private ArrayList<Double> y;
    private ArrayList<Double> z;

    public StdDev(LimelightBack vision) {
        this.vision = vision;

        addRequirements(vision);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        x = new ArrayList<>();
        y = new ArrayList<>();
        z = new ArrayList<>();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        x.add(vision.getRobotPose().getX());
        y.add(vision.getRobotPose().getY());
        z.add(vision.getRobotPose().getRotation().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Double sumX = 0d;
        for (Double i : x) {
            if (i != 0) {
                sumX += i;
            }
        }

        Double MeanX = sumX / x.size();

        Double standardDeviationX = 0.0;
        for (double num : x) {
            standardDeviationX += Math.pow(num - MeanX, 2);
        }

        standardDeviationX /= x.size();

        standardDeviationX = Math.sqrt(standardDeviationX);

        Double sumY = 0d;
        for (Double i : y) {
            if (i != 0) {
                sumY += i;
            }
        }

        Double MeanY = sumY / y.size();

        Double standardDeviationY = 0.0;
        for (double num : y) {
            standardDeviationY += Math.pow(num - MeanY, 2);
        }

        standardDeviationY /= y.size();

        standardDeviationY = Math.sqrt(standardDeviationY);

        Double sumZ = 0d;
        for (Double i : z) {
            if (i != 0) {
                sumZ += i;
            }
        }

        Double MeanZ = sumZ / z.size();

        Double standardDeviationZ = 0.0;
        for (double num : z) {
            standardDeviationZ += Math.pow(num - MeanZ, 2);
        }

        standardDeviationZ /= y.size();

        standardDeviationZ = Math.sqrt(standardDeviationZ);

        System.out.println("Vision X: " + standardDeviationX);
        System.out.println("Vision Y: " + standardDeviationY);
        System.out.println("Vision Z: " + standardDeviationZ);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
