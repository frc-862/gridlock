
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.manualLiftConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class manualLift extends CommandBase {
  /** Creates a new manualLift. */
    DoubleSupplier elevatorSpeed;
    DoubleSupplier armSpeed;
    DoubleSupplier wristSpeed;
    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;
    double armReductionConstant = 0.01;
    double elevatorReductionConstant = 0.01;
    double wristReductionConstant = 0.01;


  public manualLift(DoubleSupplier elevatorSpeed, DoubleSupplier armSpeed, DoubleSupplier wristSpeed, Arm arm, Wrist wrist, Elevator elevator ) {
    this.arm = arm;
    this.elevator = elevator;
    this.wrist = wrist;
    this.elevatorSpeed = elevatorSpeed;
    this.armSpeed = armSpeed;
    this.wristSpeed = wristSpeed;

    addRequirements(wrist,arm,elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.setPower(manualLiftConstants.ELEVATOR_SPEED_REDUCTION*elevatorSpeed.getAsDouble());
    arm.setPower(manualLiftConstants.ARM_SPEED_REDUCTION*armSpeed.getAsDouble());
    wrist.setPower(manualLiftConstants.WRIST_SPEED_REDUCTION*wristSpeed.getAsDouble());


  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    arm.stop();
    wrist.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
