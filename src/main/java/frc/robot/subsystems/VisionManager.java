
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

public class VisionManager extends SubsystemBase {

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonCamera m_camera1 = new PhotonCamera("CAM1");
  private PhotonCamera m_camera2 = new PhotonCamera("CAM2");
  private Transform3d robotToCam1;
  private Transform3d robotToCam2;

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

  private PhotonPoseEstimator m_estimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera1, robotToCam1);
  private PhotonPoseEstimator m_estimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera2, robotToCam2);
  private SwerveDrivePoseEstimator m_swerveEstimator;

  public VisionManager(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier, SwerveDrivePoseEstimator m_swerveEstimator) {
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;

    this.m_swerveEstimator = m_swerveEstimator;

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
  } 

  public Pose2d getCurrentPose(){
    return m_swerveEstimator.getEstimatedPosition();
  }

  //LIMELIGHT
  public double getTX(){
    return LimelightHelpers.getTX(getName());
  }

  public double getTY(){
    return LimelightHelpers.getTY(getName());
  }
}
