package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.LiftConstants.LiftState;

public class Lift extends SubsystemBase {

    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    public LiftState state = LiftState.stowed; 
    private Translation2d position = new Translation2d();

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator; 
        this.wrist = wrist;
        this.arm = arm;

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setState(LiftState state){
        this.state = state;
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getHeight(), ElevatorConstants.ANGLE);
    }

    public Translation2d getBarXY() {
        return new Translation2d(ArmConstants.LENGTH, new Rotation2d(arm.getAngle().getRadians()));
    }

    public Translation2d getOverallXY() {
        return ElevatorConstants.POSE_OFFSET.plus(getElevatorXY()).plus(getBarXY().plus(WristConstants.POSE_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    @Override
    public void periodic() {
        switch(state) {
            case groundCollect:
                position = LiftState.groundCollect.pose();
            break;

            case doubleSubstationCollect:
                position = LiftState.doubleSubstationCollect.pose();
            break;

            case lowScore:
                position = LiftState.lowScore.pose();
            break;

            case mediumScore:
                position = LiftState.mediumScore.pose();
            break;

            case highScore:
                position = LiftState.highScore.pose();
            break;

            case stowed:
                position = LiftState.stowed.pose();
            break;
        }

        if(isReachable(position)) {
            Translation2d currentPose = getOverallXY();
            Translation2d desiredPose = position;

            Translation2d delta = desiredPose.minus(currentPose);

            Rotation2d armAngle = delta.getAngle().minus(ElevatorConstants.ANGLE);
            double elevatorHeight = delta.getNorm() * Math.cos(armAngle.getDegrees());

            elevator.setHeight(elevatorHeight);
            arm.setAngle(armAngle);
            wrist.setAngle(new Rotation2d(armAngle.getDegrees() + 90)); //keeps the wrist parallel to the ground
        }
    }
}
