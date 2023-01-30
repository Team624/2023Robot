// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final SwerveModule m_frontLeftModule;

  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public SwerveDriveOdometry swerveOdometry;

  public PIDController skewApril_pid;
  public PIDController autoBalance_pid;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private ShuffleboardTab drivetrain_tab = Shuffleboard.getTab("Drivetrain");

  public boolean isAuton = false;
  public boolean lastPointCommand = false;
  public boolean stopAuton = false;
  public PIDController autonPoint_pidPathRotation;

  private SwerveModuleState[] lstates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public Drivetrain() {

    m_frontLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                drivetrain_tab
                    .getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.FALCON, Constants.Swerve.Mod0.FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.FALCON, Constants.Swerve.Mod0.FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.Swerve.Mod0.FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.Swerve.Mod0.FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_frontRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                drivetrain_tab
                    .getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.FALCON, Constants.Swerve.Mod1.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.FALCON, Constants.Swerve.Mod1.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.Swerve.Mod1.FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.Swerve.Mod1.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

    m_backLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                drivetrain_tab
                    .getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.FALCON, Constants.Swerve.Mod2.BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.FALCON, Constants.Swerve.Mod2.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.Swerve.Mod2.BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.Swerve.Mod2.BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

    m_backRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                drivetrain_tab
                    .getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.FALCON, Constants.Swerve.Mod3.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.FALCON, Constants.Swerve.Mod3.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.Swerve.Mod3.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.Swerve.Mod3.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    autonPoint_pidPathRotation = getRotationPathPID();
    skewApril_pid = getSkewAprilPID();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    if (fieldRelative) {

      m_chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw());

    } else {
      m_chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }
  }

  @Override
  public void periodic() {

    SwerveModulePosition[] positions = getModulePositions();

    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    if (!isAuton) {
      states = freezeLogic(states);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeftModule.set(
        states[0].speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 12.0,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 12.0,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 12.0,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * 12.0,
        states[3].angle.getRadians());

    if (isAuton) {
      swerveOdometry.update(getYaw(), positions);
    } else {
      swerveOdometry.update(getYaw(), getModulePositions());
    }

    updateROSpose();
    // autoBalance();
  }

  private SwerveModuleState getState(SwerveModule module) {
    return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
  }

  private PIDController getRotationPathPID() {
    return new PIDController(0.009, 0, 0);
  }

  private SwerveModuleState[] freezeLogic(SwerveModuleState[] current) {
    if (Math.abs(m_chassisSpeeds.omegaRadiansPerSecond)
            + Math.abs(m_chassisSpeeds.vxMetersPerSecond)
            + Math.abs(m_chassisSpeeds.vyMetersPerSecond)
        < Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND) {
      current[0].angle = lstates[0].angle;
      current[1].angle = lstates[1].angle;
      current[2].angle = lstates[2].angle;
      current[3].angle = lstates[3].angle;
    } else {
      lstates = current;
    }
    return current;
  }

  public void updateROSpose() {

    SmartDashboard.putNumber("/pose/th", getYaw().getRadians());
    SmartDashboard.putNumber("/pose/x", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("/pose/y", swerveOdometry.getPoseMeters().getY());
  }

  public void updatePoseLimelight(double[] pose) {

    Pose2d newPose = new Pose2d(pose[0], pose[1], getYaw());
    System.out.println(newPose);
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), newPose);
  }

  public void setAuton(boolean state) {
    isAuton = state;
  }

  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] pos =
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        };
    return pos;
  }

  public void setPose() {
    zeroGyroscope();
    double[] zeros = {0.0, 0.0, 0.0};
    double[] startPosition = SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(zeros);
    Rotation2d newRot = new Rotation2d(startPosition[2]);
    Pose2d newPose = new Pose2d(startPosition[0], startPosition[1], newRot);

    swerveOdometry.resetPosition(newRot, getModulePositions(), newPose);
    ahrs.setAngleAdjustment(newRot.getDegrees());
  }

  public void zeroGyroscope() {
    ahrs.setAngleAdjustment(180);
    ahrs.reset();
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public double[] getSwervePose() {

    double[] pose = {swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY()};
    return pose;
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        getYaw(),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        },
        pose);
  }

  public double normalizeAngle(double angle) {
    // Normalizes angle between (-pi and pi)
    angle %= (Math.PI * 2);
    angle = (angle + 2 * Math.PI) % (Math.PI * 2);
    if (angle > Math.PI) {
      angle -= Math.PI * 2;
    }
    return angle;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  private PIDController getSkewAprilPID() {

    return new PIDController(0.008, 0.0, 0.0);
  }

  public void stop() {
    this.drive(new Translation2d(0.0, 0.0), 0.0, false);
  }

  public AHRS getAhrs() {
    return ahrs;
  }

  public double getAngle() {
    return ahrs.getRoll();
  }
}
