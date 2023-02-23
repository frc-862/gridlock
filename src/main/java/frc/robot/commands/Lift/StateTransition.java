package frc.robot.commands.Lift;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LiftConstants.LiftPlan;
import frc.robot.Constants.LiftConstants.LiftState;

public class StateTransition {
    private double elevatorExtension;
    private Rotation2d armAngle;
    private LiftPlan plan;
    private LiftState endState;
    private Rotation2d wristAngle;

    public StateTransition(double elevatorExtension, Rotation2d armAngle, Rotation2d wristAngle, LiftPlan plan, LiftState endState) {
        this.elevatorExtension = elevatorExtension;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.plan = plan;
        this.endState = endState;
    }

    public double getElevatorExtension() {
        return elevatorExtension;
    }

    public Rotation2d getArmAngle() {
        return armAngle;
    }

    public Rotation2d getWristAngle() {
        return wristAngle;
    }

    public LiftPlan getPlan() {
        return plan;
    }

    public LiftState getEndState() {
        return endState;
    }
}
