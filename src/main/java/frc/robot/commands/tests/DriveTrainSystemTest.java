package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.command.core.TimedCommand;
import frc.thunder.swervelib.SwerveModule;

/**
 * Tests the drivetrain by driving each module at a given speed and angle
 */
public class DriveTrainSystemTest extends SequentialCommandGroup {
    /**
     * Tests the drivetrain by driving each module at a given speed and angle
     * 
     * @param drivetrain The drivetrain
     * @param module The module to drive
     * @param speed The speed to drive at
     */
    public DriveTrainSystemTest(Drivetrain drivetrain, SwerveModule module, double speed) {
        super(new WaitCommand(2), new TimedCommand(new DriveTest(drivetrain, module, speed), 2), new WaitCommand(1), new TimedCommand(new DriveTest(drivetrain, module, -speed), 2), new WaitCommand(1),
                new TimedCommand(new TurnTest(drivetrain, module, true), 2), new WaitCommand(1), new TimedCommand(new TurnTest(drivetrain, module, false), 2));
    }
}
