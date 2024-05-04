// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kPartnerControllerPort = 2;
  }
  public static class MotorConstants {  
    //HOOD
    public static final int HOOD_WRIST_MOTOR = 5;
    public static final int HOOD_MOTOR = 13;

    //ELEVATOR
    public static final int ELEVATOR_MASTER_MOTOR = 9;

    //Toaster / Shooter / Intake
    public static final int TOASTER_RIGHT_MOTOR = 1;
    public static final int TOASTER_LEFT_MOTOR = 2;
    public static final int HOLDER_MOTOR = 3;

    //Joint
    public static final int JOINT_RIGHT_MOTOR = 4;
    public static final int JOINT_LEFT_MOTOR = 7;
  }

  public static class HoodConstants{
    public static final double HOOD_SPEED = 0.7;
    public static final double kIntakeSpeed = -0.8;
  }

  public static class ElevatorConstants{
    public static final double ELEVATOR_KP = 0.65;
    public static final double ELEVATOR_KD = 0.005;
    public static final double MAX_ACCELERATION = 0.01;
    public static final double MAX_VELOCITY = 0.1;
    public static final double ELEVATOR_HEIGHT_TOLERANCE = 0.025;
    public static final double ZEROING_SPEED = -0.05;
    public static final double GEAR_RATIO = 13;
    public static final double MAX_POSITION = 26;
    public static final int kAmpSetpoint = 23;
    public static final int kSourceSetpoint = 14;
    private static final double kAntiGravity = 0.00348837;
    private static final double kAntiGravityTuner = 0.3;
    public static final double kAntiGravityMultiplier = kAntiGravity * kAntiGravityTuner;
    
  }
  public static class ToasterConstants{
    //WAIT TIME
    public static final double waitTime = 0.75; // in seconds
    //INTAKE
    public static final double intakeSpeed = -0.25;
    //SPEAKER
    public static final double SpeakerShooterSpeed = 1;
    public static final double SpeakerHolderSpeed = 0.9;
    public static final double SpeakerWaitTime = 2;
    //AMP
    public static final double AmpShooterSpeed = 0.7;
    public static final double AmpHolderSpeed = 0.5;
    public static final int kShootingSpeed = 5000;  // in rpm?
  }
  
  public static class JointConstants{
    public static final double JOINT_KP = 0.12;
    public static final double JOINT_KD = 0.025;
    public static final Rotation2d JOINT_TOLERANCE = Rotation2d.fromRotations(0.5);
    public static final double GEAR_RATIO = 179.5;

    public static final int kAmpSetpoint = 22;
    public static final double kSourceSetpoint = 24.5;

    public static final double kAntiGravityMultiplier = 0.000806452;
  }

  public static class HoodWristConstants{
    public static final double HOOD_WRIST_KP = 0.6;
    public static final double HOOD_WRIST_KD = 0.05;
    public static final double HOOD_WRIST_TOLERANCE = 0.05;
    public static final int kAmpSetpoint = 6;
    public static final int kSourceSetpoint = 10;
  }

  public static class DriveConstants {
    public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
    public static final double MAX_TANGENTIAL_VELOCITY = 6;
    public static final double kMagnitudeSlewRate = 1.8;
    public static final double kRotationalSlewRate = 2.0;
    public static final double kDirectionSlewRate = 1.2;
    public static final double kDriveDeadband = 0.1;
  }

  public static class SwerveModuleConstants{
     //DRIVE KRAKENS
    public static final int BOTTOM_RIGHT_DRIVE_PORT = 3;
    public static final int BOTTOM_LEFT_DRIVE_PORT = 1;
    public static final int TOP_RIGHT_DRIVE_PORT = 2;
    public static final int TOP_LEFT_DRIVE_PORT = 4;

    //DRIVE NEOS
    public static final int BOTTOM_RIGHT_TURN_PORT = 8;
    public static final int BOTTOM_LEFT_TURN_PORT = 10;
    public static final int TOP_RIGHT_TURN_PORT = 6;
    public static final int TOP_LEFT_TURN_PORT = 11;

    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
    
    // Drivebase measures 29x29 inches, but the swerve modules sit 12 inches in either direction from the center of the robot frame
    private static final double kWheelbaseSizeHalf = Inches.of(12).in(Meters);
    private static final Translation2d kBotRightModule = new Translation2d(-kWheelbaseSizeHalf,-kWheelbaseSizeHalf);
    private static final Translation2d kBotLeftModule  = new Translation2d(-kWheelbaseSizeHalf, kWheelbaseSizeHalf);
    private static final Translation2d kTopRightModule = new Translation2d(kWheelbaseSizeHalf, -kWheelbaseSizeHalf);
    private static final Translation2d kTopLeftModule  = new Translation2d(kWheelbaseSizeHalf,  kWheelbaseSizeHalf);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      kTopLeftModule,
      kTopRightModule,
      kBotLeftModule,
      kBotRightModule);
    
    // Gear ratios found here:
    // @{link} https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    // See here for example calculating gear ratios
    // @{link} https://www.smlease.com/entries/mechanism/gear-train-gear-ratio-torque-and-speed-calculation/
    // Drive gear ratio is calculated by taking the MK4i L2 gear ratios (the one we used)
    // and swapping out the 14 teeth drive pinion gear on stage 1 with a 16 teeth
    // drive pinion gear instead.  The resulting gear ratio is approximately 5.9028.
    public static final double TURN_GEAR_RATIO = 150.0/7;  //12.8;
    public static final double DRIVE_GEAR_RATIO = 5.9028;  // (50/16) * (17/27) * (45/15)

    public static final int BOTTOM_RIGHT_ENCODER_PORT = 12;
    public static final int BOTTOM_LEFT_ENCODER_PORT = 9;
    public static final int TOP_RIGHT_ENCODER_PORT = 11;
    public static final int TOP_LEFT_ENCODER_PORT = 10;

    public static final double BOTTOM_RIGHT_ENCODER_OFFSET = 0.309326;//0.312744;//-0.172119;
    public static final double BOTTOM_LEFT_ENCODER_OFFSET = -0.107666;//-0.103516;//0.406738;
    public static final double TOP_RIGHT_ENCODER_OFFSET = -0.155518;//-0.148926;vwww//0.361084;
    public static final double TOP_LEFT_ENCODER_OFFSET = 0.019287;//0.025146;//-0.475342

    /**
     * Note: drive kP has been multiplied by circumference because sensor to mechanism ratio
     * has been altered from DRIVE_GEAR_RATIO/CIRCUMFERENCE to just DRIVE_GEAR_RATIO.  The
     * extra division by circumference is to scale the kP value properly to the new sensor to
     * mechanism ratio.
     */
    public static final double DRIVE_KP = 0.5;
    public static final double DRIVE_KFF = 0.0001;
    public static final double DRIVE_KD = 0.001;

    /** Note:
     * Turn kP has been multiplied by the turn gear ratio because code has been added to set the
     * position conversion ratio to 1/turn_gear_ratio.  The multiplication is to properly rescale
     * the turn kP value to work.  The value of 0.5 was based off of the original code in the 
     * main (and post_vcr) branches.
     */
    public static final double TURN_KP = 0.5 * TURN_GEAR_RATIO / 2 / Math.PI;
  }

}
