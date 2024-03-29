// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
    public static final double MAX_TANGENTIAL_VELOCITY = 4;
  }
  
  public static class SwerveModuleConstants{
    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double WHEELBASE_WIDTH = 0.6604;
    public static final double WHEELBASE_LENGTH = 0.6604;

    public static final Translation2d BOTTOM_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d BOTTOM_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(TOP_LEFT_MODULE_POSITION, TOP_RIGHT_MODULE_POSITION, BOTTOM_LEFT_MODULE_POSITION, BOTTOM_RIGHT_MODULE_POSITION);

    public static final double TURN_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.75;

    public static final int BOTTOM_RIGHT_DRIVE_PORT = 1;
    public static final int BOTTOM_LEFT_DRIVE_PORT = 3;
    public static final int TOP_RIGHT_DRIVE_PORT = 7;
    public static final int TOP_LEFT_DRIVE_PORT = 5;

    public static final int BOTTOM_RIGHT_TURN_PORT = 2;
    public static final int BOTTOM_LEFT_TURN_PORT = 4;
    public static final int TOP_RIGHT_TURN_PORT = 8;
    public static final int TOP_LEFT_TURN_PORT = 6;

    public static final int BOTTOM_RIGHT_ENCODER_PORT = 12;
    public static final int BOTTOM_LEFT_ENCODER_PORT = 10;
    public static final int TOP_RIGHT_ENCODER_PORT = 9;
    public static final int TOP_LEFT_ENCODER_PORT = 11;

    public static final double BOTTOM_RIGHT_ENCODER_OFFSET = 31.3;
    public static final double BOTTOM_LEFT_ENCODER_OFFSET = -63;
    public static final double TOP_RIGHT_ENCODER_OFFSET = -128.4;
    public static final double TOP_LEFT_ENCODER_OFFSET = 167.95;

    public static final double DRIVE_KP = 0.00003;//0.00013;//0.000165;
    public static final double DRIVE_KFF = 0.00017;//0.00017;//0.00004;
    public static final double DRIVE_KD = 0.00001;//.00029;//0.00015;
    public static final double TURN_KP = 0.5;
  }
}
