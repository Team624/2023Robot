// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  public SwerveDriveOdometry swerveOdometry;

  public SwerveModule[] mSwerveMods;

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public boolean isAuton = false;
  public boolean lastPointCommand = false;
  public boolean stopAuton = false;

  public SwerveModuleState[] mainStates;

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

  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) {
      // swerveModuleStates =
      //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      //         ChassisSpeeds.fromFieldRelativeSpeeds(
      //             translation.getX(), translation.getY(), rotation, getYaw()));
      m_chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw());

    } else {
      // swerveModuleStates =
      //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      //         new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

      m_chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    //   for (SwerveModule mod : mSwerveMods) {
    //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    //   }
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void stop() {
    this.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModuleState[] m_States =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(m_States, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(m_States[mod.moduleNumber], true);
    }

    SwerveModulePosition[] positions = getModulePositions();
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    if (isAuton) {
      swerveOdometry.update(getYaw(), positions);
    } else {
      swerveOdometry.update(getYaw(), getModulePositions());
    }

    updateROSpose();
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

  public void setAuton(boolean state) {
    isAuton = state;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setPose() {
    zeroGyroscope();
    double[] zeros = {0.0, 0.0, 0.0};
    double[] startPosition = SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(zeros);
    Rotation2d newRot = new Rotation2d( startPosition[2]);
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
    return new Pose2d(swerveOdometry.getPoseMeters().getTranslation(), getYaw());
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
