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
    public static final double MAX_ANGULAR_VELOCITY = 0;
    public static final double MAX_TANGENTIAL_VELOCITY = 0;
  }
  
  public static class SwerveModuleConstants{
    public static final double WHEEL_DIAMETER_METERS = 0;
    public static final double CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double WHEELBASE_WIDTH = 0;
    public static final double WHEELBASE_LENGTH = 0;

    public static final Translation2d BOTTOM_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d BOTTOM_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d TOP_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(BOTTOM_RIGHT_MODULE_POSITION, BOTTOM_LEFT_MODULE_POSITION, TOP_RIGHT_MODULE_POSITION, TOP_LEFT_MODULE_POSITION);

    public static final double DRIVE_GEAR_RATIO = 0;
    public static final double TURN_GEAR_RATIO = 0;

    public static final int BOTTOM_RIGHT_DRIVE_PORT = 0;
    public static final int BOTTOM_LEFT_DRIVE_PORT = 0;
    public static final int TOP_RIGHT_DRIVE_PORT = 0;
    public static final int TOP_LEFT_DRIVE_PORT = 0;

    public static final int BOTTOM_RIGHT_TURN_PORT = 0;
    public static final int BOTTOM_LEFT_TURN_PORT = 0;
    public static final int TOP_RIGHT_TURN_PORT = 0;
    public static final int TOP_LEFT_TURN_PORT = 0;

    public static final int BOTTOM_RIGHT_ENCODER_PORT = 0;
    public static final int BOTTOM_LEFT_ENCODER_PORT = 0;
    public static final int TOP_RIGHT_ENCODER_PORT = 0;
    public static final int TOP_LEFT_ENCODER_PORT = 0;

    public static final double DRIVE_KP = 0;
    public static final double DRIVE_KFF = 0;
    public static final double TURN_KP = 0;

  }
}
