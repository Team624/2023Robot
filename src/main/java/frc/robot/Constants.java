// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C;
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
    public static final double driveKP = 0.15; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.01;

    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Drivetrain Constants */

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double trackWidth = 0.4953; // TODO: This must be tuned to specific robot

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double wheelBase = 0.5886; // TODO: This must be tuned to specific robot

    public static final Translation2d[] MODULE_POSITIONS = {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(MODULE_POSITIONS);

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
      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 19;
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 18;
      public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24;

      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(136.142);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              FRONT_LEFT_MODULE_DRIVE_MOTOR,
              FRONT_LEFT_MODULE_STEER_MOTOR,
              FRONT_LEFT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 10;
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 23;

      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(171.032);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              FRONT_RIGHT_MODULE_STEER_MOTOR,
              FRONT_RIGHT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific Aobot
      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;

      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(353.76);

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              BACK_LEFT_MODULE_DRIVE_MOTOR,
              BACK_LEFT_MODULE_STEER_MOTOR,
              BACK_LEFT_MODULE_STEER_ENCODER,
              angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 9;
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22;

      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(165.8496);

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
    public static final double DRIVE_CONTROLLER_X_KP = 1.4;
    public static final double DRIVE_CONTROLLER_X_KI = 0.0;
    public static final double DRIVE_CONTROLLER_X_KD = 0.0;

    // PID Controller for y alignment
    public static final double DRIVE_CONTROLLER_Y_KP = 1.4;
    public static final double DRIVE_CONTROLLER_Y_KI = 0.0;
    public static final double DRIVE_CONTROLLER_Y_KD = 0.0;

    // Profiled PID Controller for rotation
    public static final double DRIVE_CONTROLLER_ROTATION_KP = 2.0; // 0.4
    public static final double DRIVE_CONTROLLER_ROTATION_KI = 0.0;
    public static final double DRIVE_CONTROLLER_ROTATION_KD = 0.0;
    public static final double DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY =
        1.9 * Math.PI; // Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    public static final double DRIVE_CONTROLLER_ROTATION_MAX_ACCELERATION =
        Math.pow(DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY, 1.5); // 2.0

    // Rotational tolerance for autonomous paths
    public static final double AUTONOMOUS_X_TOLERANCE = 0.25;
    public static final double AUTONOMOUS_Y_TOLERANCE = 0.25;
    public static final Rotation2d AUTONOMOUS_ROTATION_TOLERANCE = Rotation2d.fromRadians(0.3);

    // TODO: Tune this
    public static final double AUTO_BALANCE_SPEED = 0.35;
    public static final double AUTO_BALANCE_GROUND_SPEED = 1.1;
    public static final double AUTO_BALANCE_GROUND_ANGLE_THRESHOLD = 14;
    public static final double AUTO_BALANCE_VELOCITY_THRESHOLD = 4.5;
    public static final double AUTO_BALANCE_GROUND_VELOCITY_THRESHOLD = 4.0;
    public static final double AUTO_BALANCE_POSITION_THRESHOLD = 3.0;

    public static final double AUTO_BALANCE_P_START = 0.051;
    public static final double AUTO_BALANCE_P_MULTIPLIER = 0.8;

    public static final double balancedAngle = 0; // The angle the robot should be at when balanced
    public static final double kP = 0.073; // The proportional constant for the PID controller
    public static final double angleSetPoint =
        0; // The angle the PID controller should try to reach
    public static final double kTurn = 0.007; // The constant for the turn PID controller
  }

  public static final class Intake {
    public static final double P = 0.01;
    public static final double I = 0;
    public static final double D = 0;
    public static final int intakeMotor = 12;
    public static final int PWMPort = 0;
  }

  public static final class Arm {

    public static final int armMotorRight = 6;

    public static final int armMotorLeft = 17;

    public static final int BORE_ENCODER_PORT = 0;

    // Absolute encoder offset
    public static final double BORE_ENCODER_OFFSET = -0.6;

    public static final Rotation2d ESTOP_TOLERANCE = Rotation2d.fromDegrees(10);

    // Profiled PID controller gains
    public static final double kP = 30.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxVelocityRadiansPerSecond = 5.45;
    public static final double kMaxAccelerationRadiansPerSecondSquared = 5.9;

    public static final double kSlowMaxVelocityRadiansPerSecond = 4.2;
    public static final double kSlowMaxAccelerationRadiansPerSecondSquared = 4.0;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.35;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Setpoints

    public static final Rotation2d ARM_SETPOINT_DOUBLE_SUBSTATION = Rotation2d.fromDegrees(100);
    public static final Rotation2d ARM_SETPOINT_BOT = Rotation2d.fromDegrees(28.5);
    public static final Rotation2d ARM_SETPOINT_PREINTAKE = Rotation2d.fromDegrees(63);
    public static final Rotation2d ARM_SETPOINT_PREHIGH_SCORE_AUTON =
        Rotation2d.fromDegrees(170); // 170 for parallel
    public static final Rotation2d ARM_SETPOINT_PREHIGH_SCORE =
        Rotation2d.fromDegrees(240); // 170 for parallel
    public static final Rotation2d ARM_SETPOINT_UPRIGHT_CONE_INTAKE = Rotation2d.fromDegrees(49.3);
    public static final Rotation2d ARM_SETPOINT_SIDE_CONE_INTAKE =
        Rotation2d.fromDegrees(41.26); // 42.8
    public static final Rotation2d ARM_SETPOINT_MID = Rotation2d.fromDegrees(272);
    public static final Rotation2d ARM_SETPOINT_HIGH = Rotation2d.fromDegrees(259);
  }

  public static final class Telescope {
    public static final double P = 11.5;
    public static final double I = 0.0;
    public static final double D = 0;
    public static final int telescopemotor = 5;

    public static final double TELESCOPE_SETPOINT_DOUBLE_SUBSTATION = 0.15;
    public static final double TELESCOPE_SETPOINT_ZERO = 0.15;
    public static final double TELESCOPE_SETPOINT_SIDE_CONE_INTAKE = 0.96; // 1.03
    public static final double TELESCOPE_SETPOINT_UPRIGHT_CONE_INTAKE = 0.79;
    public static final double TELESCOPE_SETPOINT_MID = 0.15;
    public static final double TELESCOPE_SETPOINT_HIGH = 1.11;

    // FF constants
    public static final double kS = 0.0;
    public static final double kG = 0.0;

    public static final double kV = 0.0;
  }

  public static final class Wrist {
    public static final double P = 15.1;
    public static final double I = 0.1;
    public static final double D = 0.0;
    public static final int WristMotor = 13;

    public static final Rotation2d WRIST_STOP_MAX = new Rotation2d();
    public static final Rotation2d WRIST_STOP_MIN = new Rotation2d();

    public static final double boreEncoderOffset = -0.844;
    // -0.507

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final Rotation2d wrist_upright_cone_intake = Rotation2d.fromDegrees(180);
    public static final Rotation2d wrist_zero = Rotation2d.fromDegrees(1);
    public static final Rotation2d wrist_cone_intake = Rotation2d.fromDegrees(90);
    public static final Rotation2d wrist_cone_leftScore = Rotation2d.fromDegrees(90);
  }

  public static final class Shooter {
    public static final int shooterMotorLeft = 15;
    public static final int shooterMotorRight = 16;
    public static final double HighScoreSpeed = -0.42;
    public static final double MidScoreSpeed = -0.3;
    public static final double LowScoreSpeed = -1.0;
    public static final double IntakeSpeed = 0.3;
    public static final double ChargerSpeed = 0.0;
    public static final double OverBumpSpeed = 0.0;
    public static final double FlatCommunitySpeed = 0.0;

    public static final double P = 0.3;
    public static final double I = 0.3;
    public static final double D = 0.3;
    public static final double Izone = 0.3;
  }

  public static final class Hood {
    public static final int hoodMotor = 14;

    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double BORE_ENCODER_OFFSET = 0.1;
    // 0.1

    public static final Rotation2d Hood_Upright_Setpoint = Rotation2d.fromDegrees(3.5 + 36);
    public static final Rotation2d Hood_High_Setpoint = Rotation2d.fromDegrees(33.2 + 36);
    public static final Rotation2d Hood_Mid_Setpoint = Rotation2d.fromDegrees(31.8 + 36);
    public static final Rotation2d Hood_Intake_Setpoint = Rotation2d.fromDegrees(108 + 36);
    public static final Rotation2d Hood_Hybrid_Setpoint = Rotation2d.fromDegrees(51 + 36);

    public static final Rotation2d Hood_Charger_Setpoint = Rotation2d.fromDegrees(51 + 36);
    public static final Rotation2d Hood_Over_Bump_Setpoint = Rotation2d.fromDegrees(51 + 36);
    public static final Rotation2d Hood_Flat_Community_setpoint = Rotation2d.fromDegrees(51 + 36);
  }

  public static final class LEDs {
    public static final int LEDS_ID = 0x30;
    public static final I2C.Port LEDS_PORT = I2C.Port.kMXP;
  }

  public static final class Limelight {
    public static final double[] tagLocations = {
      0.0, -6.94659, -5.27019, -3.59379, -1.26839, -1.26839, -3.59379, -5.27019, -6.94659
    };

    public static final int numValuesAveraged = 5;

    public static final double kTranslationP = 10.8;
    public static final double kTranslationI = 0.3;
    public static final double kTranslationD = 0.0;

    public static final double kRotationP = 10;
  }
}
