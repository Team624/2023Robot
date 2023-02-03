// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
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
  public static final class Swerve {

    public static final double Max_Voltage = 12.0;

    public static final double TRANSLATION_TUNING_CONSTANT = 1;
    public static final double PATH_POINT_RANGE = 0.1;

    public static final double DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER = .8;
    public static final double DRIVETRAIN_INPUT_ROTATION_MULTIPLIER = .4;

    public static final double DRIVETRAIN_INPUT_SPEED_MULTIPLIER = 1.4;
    public static final double DRIVETRAIN_INPUT_DEADBAND = .05;

    /* Drivetrain Constants */

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double trackWidth = 0.466598; // TODO: This must be tuned to specific robot

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double wheelBase = 0.466598; // TODO: This must be tuned to specific robot

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0));

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6379.0
            / 60.0
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
      public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;

      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(66.26);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;

      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(97.64);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;

      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(286.16);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;

      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(232.99);
    }
  }
  public static final class Arm{
    public static final int ARM_MOTOR=0;
    public static final double P=0;
    public static final double I=0;
    public static final double D=0;
    public static final double FF=0;
  }
  public static final class Pivot{
    public static final int PIVOT_MOTOR=0;
    public static final double P=0;
    public static final double I=0;
    public static final double D=0;
    public static final double FF=0;
  }
}
