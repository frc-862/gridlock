// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonBack extends SubsystemBase {

    public String photonName;
    PhotonCamera cam = new PhotonCamera(photonName);
    boolean hasTarget = false;

  /** Creates a new PhotonBack. */
  public PhotonBack(String photonName) {
    this.photonName = photonName;

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public PhotonPipelineResult getPipelineResult() {
    return cam.getLatestResult();
  }

  public double getYaw(){
    return getPipelineResult().getBestTarget().getYaw();
  }

  public double getPitch(){
    return getPipelineResult().getBestTarget().getPitch();
  }

  public double getSkew(){
    return getPipelineResult().getBestTarget().getSkew();
  }

}
