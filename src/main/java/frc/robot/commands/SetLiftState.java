package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.DrivetrainConstants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.*;

public class SetLiftState extends CommandBase {
    Elevator elevator;
    Arm arm;
    Supplier<Translation2d> pose;

    public SetLiftState(Elevator elevator, Arm arm, LiftState liftState) {
        this(elevator, arm, () -> liftState.pose());
    }

    public SetLiftState(Elevator elevator, Arm arm, DoubleSupplier x, DoubleSupplier y) {
        this(elevator, arm, () -> (new Translation2d(x.getAsDouble(), y.getAsDouble())));
    }

    public SetLiftState(Elevator elevator, Arm arm, Supplier<Translation2d> pose) {
        this.elevator = elevator;
        this.arm = arm;
        this.pose = pose;
        addRequirements(elevator, arm);
    }

    public Translation2d getOverallXY() {
        return ElevatorConstants.POSE_OFFSET.plus(elevator.getElevatorXY()).plus(arm.getArmXY().plus(WristConstants.POSE_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    @Override
    public void initialize() {
        //TODO: decide if we want this to be called once and stop
        //TODO: or if we want it to be more like a default command
        if (isReachable(pose.get())) {
            Translation2d currentPose = getOverallXY();
            Translation2d desiredPose = this.pose.get();

            Translation2d delta = desiredPose.minus(currentPose);

            Rotation2d barAngle = delta.getAngle().minus(ElevatorConstants.ANGLE);
            double elevatorHeight = delta.getNorm() * Math.cos(barAngle.getRadians());

            elevator.setHeight(elevatorHeight);
            arm.setAngle(barAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.onTarget() && arm.onTarget();
    }
}
