package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class Squish extends CommandBase {
    private Arm arm;
    private boolean enable = false;
    
    /**
     * Makes the arm lower current to allow squishing against wall
     * @param arm arm subsystem
     * @param enable true = on, fase = of
     */
    public Squish(Arm arm) {
        this.arm = arm;
    }
    
    @Override
    public void initialize() {
        enable = arm.getSquishToggle();
    }

    @Override
    public void execute() {
        if(enable){
            arm.setCurrentLimit(5);
        } else {
            arm.setCurrentLimit(ArmConstants.CURRENT_LIMIT);
        }

        enable = arm.getSquishToggle();
    }
}
