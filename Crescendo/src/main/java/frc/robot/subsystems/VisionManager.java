// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;


public class VisionManager extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonCamera m_camera1 = new PhotonCamera("joemama");
  private PhotonCamera m_camera2 = new PhotonCamera("Jackson");
  private PhotonPipelineResult results1;
  private PhotonPipelineResult results2;
  private Pose3d robotPose;
  private PhotonTrackedTarget target1;
  private PhotonTrackedTarget target2;
  Transform3d robotToCam1;
  Transform3d robotToCam2;

  PhotonPoseEstimator m_estimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera1, robotToCam1);
  PhotonPoseEstimator m_estimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera2, robotToCam2);
  SwerveDrivePoseEstimator m_swerveEstimator;
  
  public VisionManager(Rotation2d gyroAngle, SwerveModulePosition[] swervePositions) {

    m_swerveEstimator = new SwerveDrivePoseEstimator(SwerveModuleConstants.DRIVE_KINEMATICS, gyroAngle, swervePositions, null);
  }

  @Override
  public void periodic() {
    update(null, null);
  }

  public void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){
    m_swerveEstimator.update(gyro.getRotation2d(), modulePositions);
    
    results1 = m_camera1.getLatestResult();
    results2 = m_camera2.getLatestResult();
    
    //add smth for that speific aiming tag on the speaker
    if(results1.hasTargets()){
      target1 = results1.getBestTarget();
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target1.getBestCameraToTarget(), 
      aprilTagFieldLayout.getTagPose(target1.getFiducialId()).get(), new Transform3d());
    }

    if(results2.hasTargets()){
      target2 = results2.getBestTarget();
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target1.getBestCameraToTarget(), 
      aprilTagFieldLayout.getTagPose(target1.getFiducialId()).get(), new Transform3d());
    }
    m_swerveEstimator.addVisionMeasurement(null, 0);
  }



  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    AprilTagPoseEstimate estimate = new AprilTagPoseEstimate(null, null, 0, 0)   
    m_estimator1.setReferencePose(prevEstimatedRobotPose);
        return m_estimator1.update();
    }


}
