// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SdsModuleConfigurations;
import frc.lib.util.SwerveModuleConstants;

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

    public static final double driveKS = (0.21819 / 12);
    public static final double driveKV = (0.69858 / 12);
    public static final double driveKA = (0.19647 / 12);

    public static final COTSFalconSwerveConstants
        chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

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

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(247);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              FRONT_LEFT_MODULE_DRIVE_MOTOR,
              FRONT_LEFT_MODULE_STEER_MOTOR,
              FRONT_LEFT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 19;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;

      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(97.64);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(279.57);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              FRONT_RIGHT_MODULE_STEER_MOTOR,
              FRONT_RIGHT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;

      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(286.16);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.6);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              BACK_LEFT_MODULE_DRIVE_MOTOR,
              BACK_LEFT_MODULE_STEER_MOTOR,
              BACK_LEFT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;

      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(232.99);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(53);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              BACK_RIGHT_MODULE_DRIVE_MOTOR,
              BACK_RIGHT_MODULE_STEER_MOTOR,
              BACK_RIGHT_MODULE_STEER_ENCODER,
              angleOffset);
    }
  }

  public static final class Autonomous {
    // PID Controler for x alignment
    public static final double DRIVE_CONTROLLER_X_KP = 3.0;
    public static final double DRIVE_CONTROLLER_X_KI = 0.0;
    public static final double DRIVE_CONTROLLER_X_KD = 0.0;

    // PID Controller for y alignment
    public static final double DRIVE_CONTROLLER_Y_KP = 3.0;
    public static final double DRIVE_CONTROLLER_Y_KI = 0.0;
    public static final double DRIVE_CONTROLLER_Y_KD = 0.0;

    // Profiled PID Controller for rotation
    public static final double DRIVE_CONTROLLER_ROTATION_KP = .2685; // 0.4
    public static final double DRIVE_CONTROLLER_ROTATION_KI = 0.0;
    public static final double DRIVE_CONTROLLER_ROTATION_KD = 0.0;
    public static final double DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY =
        3.0 * Math.PI; // Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    public static final double DRIVE_CONTROLLER_ROTATION_MAX_ACCELERATION =
        Math.pow(DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY, 2); // 2.0

    // Rotational tolerance for autonomous paths
    public static final double AUTONOMOUS_X_TOLERANCE = 0.2;
    public static final double AUTONOMOUS_Y_TOLERANCE = 0.2;
    public static final Rotation2d AUTONOMOUS_ROTATION_TOLERANCE = Rotation2d.fromRadians(0.5);
  }
}
