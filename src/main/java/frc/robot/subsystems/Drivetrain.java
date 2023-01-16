// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public static final double MAX_VOLTAGE = 12.0;

  public SwerveDriveOdometry swerveOdometry;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] lstates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public SwerveModule[] mSwerveMods;

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  public boolean isAuton = false;
  public boolean lastPointCommand = false;
  public boolean stopAuton = false;
  public PIDController autonPoint_pidPathRotation;

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
  }

  public void drive(
      // Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop
      ChassisSpeeds chassisSpeeds) {

    // SwerveModuleState[] swerveModuleStates;
    // if (fieldRelative) {
    //   swerveModuleStates =
    //       Constants.Swerve.swerveKinematics.toSwerveModuleStates(
    //           ChassisSpeeds.fromFieldRelativeSpeeds(
    //               translation.getX(), translation.getY(), rotation, getYaw()));

    // } else {
    //   swerveModuleStates =
    //       Constants.Swerve.swerveKinematics.toSwerveModuleStates(
    //           new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    // }
    // // SwerveModuleState[] swerveModuleStates =
    // //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
    // //         fieldRelative
    // //             ? ChassisSpeeds.fromFieldRelativeSpeeds(
    // //                 translation.getX(), translation.getY(), rotation, getYaw())
    // //             : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // for (SwerveModule mod : mSwerveMods) {
    //   mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    // }

    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveModulePosition[] positions = getModulePositions();
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    // mSwerveMods[0].set(states[0].speedMetersPerSecond /
    // Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //                             states[0].angle.getRadians());
    // mSwerveMods[1].set(states[1].speedMetersPerSecond /
    // Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //                 states[1].angle.getRadians());
    // mSwerveMods[2].set(states[2].speedMetersPerSecond /
    // Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //                 states[2].angle.getRadians());
    // mSwerveMods[3].set(states[3].speedMetersPerSecond /
    // Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    //                 states[3].angle.getRadians());

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber], true);
    }

    if (isAuton) {
      swerveOdometry.update(getYaw(), positions);
    } else {
      swerveOdometry.update(getYaw(), getModulePositions());
    }

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    updateROSPose();
  }

  private PIDController getRotationPathPID() {
    // .06, 0, 0
    return new PIDController(.009, 0, 0);
  }

  public void setAuton(boolean state) {
    isAuton = state;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setPose() {
    zeroGyroscope();
    double[] zeros = {0.0, 0.0, 0.0};
    double[] startPosition = SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(zeros);
    Rotation2d newRot = new Rotation2d(-startPosition[2]);
    Pose2d newPose = new Pose2d(startPosition[0], startPosition[1], newRot);
    // swerveOdometry.resetPosition(newPose, newRot);
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

  public void updateROSPose() {
    SmartDashboard.putNumber("/pose/th", getYaw().getRadians());
    SmartDashboard.putNumber("/pose/x", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("/pose/y", swerveOdometry.getPoseMeters().getY());
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  public double normalizeNuclearBombs(double angle) {
    // Normalizes angle between (-pi and pi)
    angle %= (Math.PI * 2);
    angle = (angle + 2 * Math.PI) % (Math.PI * 2);
    if (angle > Math.PI) {
      angle -= Math.PI * 2;
    }
    return angle;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }
}
