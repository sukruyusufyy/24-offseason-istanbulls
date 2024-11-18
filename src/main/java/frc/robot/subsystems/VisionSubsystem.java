// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class VisionSubsystem extends SubsystemBase {
  // PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  // PhotonCamera obje = new PhotonCamera("TANDBERG_Video");
  PhotonCamera target_cam;
  PhotonCamera object_cam;

  int targetID = 0;

  public VisionSubsystem() {
    target_cam = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    object_cam = new PhotonCamera("TANDBERG_Video");
  }

  public boolean isSeeingObject() {
    var result = object_cam.getLatestResult();
    // SmartDashboard.putBoolean("is Seeing Object", result.hasTargets());
    return result.hasTargets();
  }

  public double getObjectYaw() {
    var result = object_cam.getLatestResult();

    if (result.hasTargets()) {
      // this returns the first element of the list
      PhotonTrackedTarget target = result.getBestTarget();

      double yaw = target.getYaw();
      // SmartDashboard.putNumber("Best Object Yaw", yaw);
      return yaw;
    }
    return -1;
  }

  public boolean isSpeakerAvailable() {
    return getDistanceToSpeaker() > 0;
  }

  public double getDistanceToSpeaker() {
    var result = target_cam.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 4 || targetID == 7) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double distance = target.getBestCameraToTarget().getX() * 100;
        SmartDashboard.putNumber("Speaker Distance", distance);
        
        return distance;
      }
    }
    return -1;
  }

  public double getPitchToSpeaker() {
    var result = target_cam.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 4 || targetID == 7) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double pitch = target.getPitch();
        SmartDashboard.putNumber("Speaker Pitch", pitch);
        
        return pitch;
      }
    }
    return 0;
  }

  public boolean isAmpAvailable() {
    return getDistanceToAmp() > 0;
  }

  public double getDistanceToAmp() {
    var result = target_cam.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 5) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double distance = target.getBestCameraToTarget().getX() * 100;
        SmartDashboard.putNumber("Amp Distance", distance);
        return distance;
      }
    }
    else{

    }
    return -1;
  }
}
