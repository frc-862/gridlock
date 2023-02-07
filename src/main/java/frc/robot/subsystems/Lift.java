package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.DrivetrainConstants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;
import frc.robot.Constants.DrivetrainConstants.WristConstants;

public class Lift extends SubsystemBase {

    public enum States {
        groundCollect,
        doubleSubstationCollect,
        lowScore,
        mediumScore,
        highScore,
        stowed
    }

    private Elevator elevator;
    private Wrist wrist;
    private Arm arm;

    public States state = States.stowed; 
    private Translation2d position = new Translation2d();

    public Lift(Elevator elevator, Wrist wrist, Arm arm) {
        this.elevator = elevator; 
        this.wrist = wrist;
        this.arm = arm;

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setState(States state){
        this.state = state;
    }

    public Translation2d getElevatorXY() {
        return new Translation2d(elevator.getHeight(), ElevatorConstants.ANGLE);
    }

    public Translation2d getBarXY() {
        return new Translation2d(ArmConstants.LENGTH, new Rotation2d(Math.toRadians(arm.getAngle())));
    }

    public Translation2d getOverallXY() {
        return ElevatorConstants.OFFSET.plus(getElevatorXY()).plus(getBarXY().plus(WristConstants.COLLECTOR_OFFSET));
    }

    public Boolean isReachable(Translation2d pose) {
        return LiftConstants.BOUNDING_BOX.contains(pose.getX(), pose.getY());
    }

    @Override
    public void periodic() {
        switch(state) {
            case groundCollect:
                position = LiftConstants.GROUND_COLLECT;
            break;

            case doubleSubstationCollect:
                position = LiftConstants.DOUBLE_SUBSTATION_COLLECT;
            break;

            case lowScore:
                position = LiftConstants.LOW_SCORE;
            break;

            case mediumScore:
                position = LiftConstants.MEDIUM_SCORE;
            break;

            case highScore:
                position = LiftConstants.HIGH_SCORE;
            break;

            case stowed:
                position = LiftConstants.STOWED;
            break;
        }

        if(isReachable(position)) {
            Translation2d currentPose = getOverallXY();
            Translation2d desiredPose = position;

            Translation2d delta = desiredPose.minus(currentPose);

            Rotation2d armAngle = delta.getAngle().minus(ElevatorConstants.ANGLE);
            double elevatorHeight = delta.getNorm() * Math.cos(armAngle.getDegrees());

            elevator.setHeight(elevatorHeight);
            arm.setAngle(armAngle.getDegrees());
            wrist.setAngle(armAngle.getDegrees() + 90); //keeps the wrist parallel to the ground
        }
    }
}
