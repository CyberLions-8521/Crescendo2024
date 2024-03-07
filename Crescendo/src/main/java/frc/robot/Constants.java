// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class MotorConstants {  
    //Wrist
    //public static final int INDEXER_WRIST_MOTOR = 0;
    public static final int HOOD_WRIST_MOTOR = 5;

    //Hood
    public static final int HOOD_MOTOR = 13;

    //Indexer
    //public static final int INDEXER_MOTOR = 0;

    //Elevator
    public static final int ELEVATOR_MASTER_MOTOR = 9;

    //Toaster / Shooter / Intake
    public static final int TOASTER_RIGHT_MOTOR = 1;
    public static final int TOASTER_LEFT_MOTOR = 2;
    public static final int HOLDER_MOTOR = 3;

    //Joint
    public static final int JOINT_RIGHT_MOTOR = 4;
    public static final int JOINT_LEFT_MOTOR = 7;
  }

  //WRIST CONSTANTS
  public static class WristConstants{
    public static final double WRIST_KP = 0;
    public static final double WRIST_KD = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_VELOCITY = 0;
  }
  public static class HoodConstants{
    public static final double HOOD_SPEED = 0.8;
  }

  public static class ElevatorConstants{
    public static final double ELEVATOR_KP = 0;
    public static final double ELEVATOR_KD = 0;

    public static final double MAX_ACCELERATION = 0.01;
    public static final double MAX_VELOCITY = 0.1;

    public static final double ELEVATOR_HEIGHT_TOLERANCE = 1;
    public static final double ZEROING_SPEED = -0.05;
    public static final double GEAR_RATIO = 13;

    public static final double MAX_POSITION = 26;
    public static final double AMP_POSITION = 0;
    public static final double SPEAKER_POSITION = 0;
    public static final double SOURCE_POSITION = 0;
    public static final double TRAP_POSITION = 0;
    public static final double GROUND_INTAKE_POSITION = 0;
    
  }
  public static class ToasterConstants{
    //WAIT TIME
    public static final double waitTime = 0.75;

    //INTAKE
    public static final double intakeSpeed = -0.25;

    //SPEAKER
    public static final double SpeakerShooterSpeed = .9;
    public static final double SpeakerHolderSpeed = 0.9;
    public static final double SpeakerWaitTime = 0.75;

    //AMP
    public static final double AmpShooterSpeed = 0.3;
    public static final double AmpHolderSpeed = 0.3;
  }
  
  public static class JointConstants{
    public static final double JOINT_KP = 0;
    public static final double JOINT_KD = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_VELOCITY = 0;
    public static final Rotation2d AMP_POSITION = Rotation2d.fromRotations(0);
    public static final Rotation2d SPEAKER_POSITION = Rotation2d.fromRotations(0);
    public static final Rotation2d SOURCE_POSITION = Rotation2d.fromRotations(0);
    public static final Rotation2d TRAP_POSITION = Rotation2d.fromRotations(0);
    public static final Rotation2d GROUND_INTAKE_POSITION = Rotation2d.fromRotations(0);
  }

  public static class HoodWristConstants{
    public static final double HOOD_WRIST_KP = 0;
    public static final double HOOD_WRIST_KD = 0;
  }

  public static class DriveConstants {
    public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
    public static final double MAX_TANGENTIAL_VELOCITY = 4;
  }

  public static class SwerveModuleConstants{
    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double WHEELBASE_WIDTH = 0.7366;//0.6604;
    public static final double WHEELBASE_LENGTH = 0.7366;//0.6604;

    public static final Translation2d BOTTOM_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d BOTTOM_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(WHEELBASE_WIDTH / 2, WHEELBASE_LENGTH / 2),
      new Translation2d(WHEELBASE_WIDTH / 2, -WHEELBASE_LENGTH / 2),
      new Translation2d(-WHEELBASE_WIDTH / 2, WHEELBASE_LENGTH / 2),
      new Translation2d(-WHEELBASE_WIDTH / 2, -WHEELBASE_LENGTH / 2));


    public static final double TURN_GEAR_RATIO = 150.0/7;//12.8;
    public static final double DRIVE_GEAR_RATIO = 5.9028;//8.14;//6.12;//6.75;

    //KRAKENS
    public static final int BOTTOM_RIGHT_DRIVE_PORT = 3;
    public static final int BOTTOM_LEFT_DRIVE_PORT = 1;
    public static final int TOP_RIGHT_DRIVE_PORT = 2;
    public static final int TOP_LEFT_DRIVE_PORT = 4;

    //NEOS
    public static final int BOTTOM_RIGHT_TURN_PORT = 8;
    public static final int BOTTOM_LEFT_TURN_PORT = 10;
    public static final int TOP_RIGHT_TURN_PORT = 6;
    public static final int TOP_LEFT_TURN_PORT = 11;

    public static final int BOTTOM_RIGHT_ENCODER_PORT = 12;
    public static final int BOTTOM_LEFT_ENCODER_PORT = 9;
    public static final int TOP_RIGHT_ENCODER_PORT = 11;
    public static final int TOP_LEFT_ENCODER_PORT = 10;

    public static final double BOTTOM_RIGHT_ENCODER_OFFSET = -0.133545 ;//-0.3*360;//107.9;//0.444824*360;//0.440918*360;
    public static final double BOTTOM_LEFT_ENCODER_OFFSET = -0.392090   ;//-82;//0.409424*360;//-0.104492 *360;//-0.395508*360;
    public static final double TOP_RIGHT_ENCODER_OFFSET = 0.145020 ;//-61.17;//-0.341309*360;//-0.359131*360;
    public static final double TOP_LEFT_ENCODER_OFFSET = -0.479492  ;//10.45;//-0.023438*360;//-0.006836*360;

    public static final double DRIVE_KP = 0.01;//0.00015;//0.00003;//0.00013;//0.000165;
    public static final double DRIVE_KFF = 0.0105;//0.000195;//0.00017;//0.00017;//0.00004;
    public static final double DRIVE_KD = 0;//0.001;//0.00001;//.00029;//0.00015;
    public static final double TURN_KP = 0.1;//0.5;

    public static final double LOWER_BOUND = 0.0;
    public static final double UPPER_BOUND = 360.0;
  }

}
