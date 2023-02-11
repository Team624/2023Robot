// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public SwerveDriveOdometry swerveOdometry;

  private boolean m_isOpenLoop;

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

  public SwerveModule[] mSwerveMods;

  public Drivetrain() {
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    autonPoint_pidPathRotation = getRotationPathPID();
    skewApril_pid = getSkewAprilPID();
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if (fieldRelative) {

      m_chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw());

    } else {
      m_chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    m_isOpenLoop = isOpenLoop;
  }

  @Override
  public void periodic() {

    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveModulePosition[] positions = getModulePositions();

    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    if (!isAuton) {
      states = freezeLogic(states);

      for (SwerveModule mod : mSwerveMods) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      }
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], m_isOpenLoop);
    }

    if (isAuton) {
      swerveOdometry.update(getYaw(), positions);
    } else {
      swerveOdometry.update(getYaw(), getModulePositions());
    }

    updateROSpose();
    // autoBalance();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
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

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
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
    ahrs.setAngleAdjustment(0);
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
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
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
    this.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
  }

  public AHRS getAhrs() {
    return ahrs;
  }

  public double getAngle() {
    return ahrs.getRoll();
  }
}
