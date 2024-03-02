// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

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
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Barcode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;


public class VisionManager extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonCamera m_camera1 = new PhotonCamera("joemama");
  private PhotonCamera m_camera2 = new PhotonCamera("Jackson");
  private PhotonPipelineResult results;
  private PhotonTrackedTarget target;
  private Pose3d robotPose;
  private Transform3d robotToCam1;
  private Transform3d robotToCam2;
  private double imageCaptureTime;

  private AprilTagPoseEstimate estimator;

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

  private PhotonPoseEstimator m_estimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera1, robotToCam1);
  private PhotonPoseEstimator m_estimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera2, robotToCam2);
  private SwerveDrivePoseEstimator m_swerveEstimator;

  
  public VisionManager(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier, SwerveDrivePoseEstimator m_swerveEstimator) {
    
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;

    this.m_swerveEstimator = m_swerveEstimator;
    // m_swerveEstimator = new SwerveDrivePoseEstimator(SwerveModuleConstants.DRIVE_KINEMATICS, rotationSupplier.get(), modulePositionSupplier.get(), null);

    m_estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    update();
  }

  public void update(){
    m_swerveEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());
    updateVisionPoseEstimate(m_estimator1);
    updateVisionPoseEstimate(m_estimator2);
  }
  
  private void updateVisionPoseEstimate(PhotonPoseEstimator m_estimator){
    Optional<EstimatedRobotPose> estimatedPose = m_estimator.update();
    if(estimatedPose.isPresent()){
      m_swerveEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
    }
    // if(m_estimator.update() != null){
    //   results = m_camera1.getLatestResult();  
    //   target = results.getBestTarget();
    //   target.getPoseAmbiguity();
    //   imageCaptureTime = results.getTimestampSeconds();
    //   robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), 
    //   aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), new Transform3d());
    //   m_swerveEstimator.addVisionMeasurement(robotPose.toPose2d(), imageCaptureTime);
    // }
    //or
  } 


  //aim assist
  // public double getYawSpeakerTag(){
  //   //Yaw = right of the april tag
  //   results = m_camera1.getLatestResult();  
  //   target = results.getBestTarget();
  //   if(target.getFiducialId() == 1){
  //     return target.getYaw();
  //   }
  //   return 0;
  // }
  // public double getPitchSpeakerTag(){
  //   //pitch = up of the april tag
  //   results = m_camera1.getLatestResult();  
  //   target = results.getBestTarget();
  //   if(target.getFiducialId() == 1){
  //     return target.getPitch();
  //   }    
  //   return 0;
  // }
  public Pose2d getCurrentPose(){
    return m_swerveEstimator.getEstimatedPosition();
  }
}
