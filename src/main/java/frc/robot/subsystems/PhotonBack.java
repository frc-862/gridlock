// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;


public class PhotonBack extends SubsystemBase {
    PhotonCamera photonCamera;
    PhotonPoseEstimator photonPoseEstimator;

    public String photonName;
    PhotonCamera cam = new PhotonCamera(photonName);
    boolean loggingStarted = false;

    private LightningShuffleboardPeriodic periodicShuffleboard;

  /** Creates a new PhotonBack. */
  public PhotonBack(String photonName) {
    this.photonName = photonName;

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!loggingStarted && hasVision()){
        initializeLogging();
    }

    if (hasVision() && loggingStarted){
        periodicShuffleboard.loop();
    }
  }

  public boolean hasVision() {
    return getPipelineResult().getTargets() != null;
  }

  public PhotonPipelineResult getPipelineResult() {
    return cam.getLatestResult();
  }

  public PhotonTrackedTarget getTrackedTarget() {
    return getPipelineResult().getBestTarget();
  }

  public double getYaw(){
    return getTrackedTarget().getYaw();
  }

  public double getPitch(){
    return getTrackedTarget().getPitch();
  }

  public double getSkew(){
    return getTrackedTarget().getSkew();
  }

  public double getTargetId(){
    return getTrackedTarget().getFiducialId();
  }

  @SuppressWarnings("unchecked")
  private void initializeLogging() {
    periodicShuffleboard = new LightningShuffleboardPeriodic("Vision", 0.17, 
        new Pair<String, Object>(photonName + "Skew", (DoubleSupplier) () -> getSkew()), 
        new Pair<String, Object>(photonName + "Pitch", (DoubleSupplier) () -> getPitch()),
        new Pair<String, Object>(photonName + "Yaw", (DoubleSupplier) () -> getYaw()),
        new Pair<String, Object>(photonName + "TrackedTarget", (DoubleSupplier) () -> getTargetId()),
        new Pair<String, Object>(photonName + "Has Vision", (BooleanSupplier) () -> hasVision()));
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}