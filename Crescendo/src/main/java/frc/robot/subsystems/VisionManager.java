// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class VisionManager extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonCamera m_camera1 = new PhotonCamera("joemama");
  private PhotonCamera m_camera2 = new PhotonCamera("Jackson");
  private PhotonPipelineResult results1;
  private PhotonPipelineResult results2;
  private boolean hasTargets1;
  private boolean hasTargets2;
  Pose3d robotPose;
  PhotonTrackedTarget target1;
  PhotonTrackedTarget target2;

  public VisionManager() {

  }

  @Override
  public void periodic() {
    results1 = m_camera1.getLatestResult();
    results2 = m_camera2.getLatestResult();
    hasTargets1 = results1.hasTargets();
    hasTargets2 = results2.hasTargets();
    
    if(hasTargets1){
      target1 = results1.getBestTarget();
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target1.getBestCameraToTarget(), 
      aprilTagFieldLayout.getTagPose(target1.getFiducialId()).get(), new Transform3d());
    }

    if(hasTargets2){
      target2 = results2.getBestTarget();
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target1.getBestCameraToTarget(), 
      aprilTagFieldLayout.getTagPose(target1.getFiducialId()).get(), new Transform3d());
    }
    
  }

  @Override
  public void simulationPeriodic() {
  }
}
