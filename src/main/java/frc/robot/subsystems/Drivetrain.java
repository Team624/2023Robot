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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private Field2d field = new Field2d();

  public boolean isAuton = false;

  private SwerveModuleState[] m_states =
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

    skewApril_pid = getSkewAprilPID();

    drivetrain_tab.add(field).withSize(4, 3);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    drive(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        fieldRelative,
        isOpenLoop);
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean isOpenLoop) {
    if (fieldRelative) {
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getYaw());
    } else {
      m_chassisSpeeds = chassisSpeeds;
    }

    m_isOpenLoop = isOpenLoop;

    m_states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false, false);
  }

  @Override
  public void periodic() {
    if (!isAuton) m_states = freezeLogic(m_states);

    setModuleStates();

    swerveOdometry.update(getYaw(), getModulePositions());

    updateNT();
  }

  private void updateNT() {
    ChassisSpeeds currentChassisSpeeds = getChassisSpeeds(true);
    Pose2d currentPose = getPose();

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("X Velocity", currentChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("X Velocity Setpoint", -m_chassisSpeeds.vxMetersPerSecond);

    SmartDashboard.putNumber("Y Velocity", currentChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Y Velocity Setpoint", -m_chassisSpeeds.vyMetersPerSecond);

    SmartDashboard.putNumber("Rotation Velocity", currentChassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Rotation Velocity Setpoint", m_chassisSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("/pose/th", getYaw().getRadians());
    SmartDashboard.putNumber("/pose/x", currentPose.getX());
    SmartDashboard.putNumber("/pose/y", currentPose.getY());

    field.setRobotPose(
        new Pose2d(currentPose.getX(), 8.0137 + currentPose.getY(), getYaw()));
  }

  public void setModuleStates() {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(m_states[mod.moduleNumber], m_isOpenLoop);
    }
  }

  public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
        Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());

    if (!fieldRelative) return chassisSpeeds;

    // Convert robot relative to field relative
    return new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond * getYaw().getCos()
            - chassisSpeeds.vyMetersPerSecond * getYaw().getSin(),
        chassisSpeeds.vyMetersPerSecond * getYaw().getCos()
            + chassisSpeeds.vxMetersPerSecond * getYaw().getSin(),
        chassisSpeeds.omegaRadiansPerSecond);
  }

  private SwerveModuleState[] freezeLogic(SwerveModuleState[] current) {
    if (Math.abs(m_chassisSpeeds.omegaRadiansPerSecond)
            + Math.abs(m_chassisSpeeds.vxMetersPerSecond)
            + Math.abs(m_chassisSpeeds.vyMetersPerSecond)
        < Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND) {
      current[0].angle = m_states[0].angle;
      current[1].angle = m_states[1].angle;
      current[2].angle = m_states[2].angle;
      current[3].angle = m_states[3].angle;
    } else {
      m_states = current;
    }
    return current;
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
    double[] startPosition =
        SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(new double[3]);
    Rotation2d newRot = new Rotation2d(-startPosition[2]);
    Pose2d newPose = new Pose2d(startPosition[0], startPosition[1], newRot.times(-1));

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

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  private PIDController getSkewAprilPID() {
    return new PIDController(0.008, 0.0, 0.0);
  }

  public AHRS getAhrs() {
    return ahrs;
  }

  public double getRoll() {
    return ahrs.getRoll();
  }
}
