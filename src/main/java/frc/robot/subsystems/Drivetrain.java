// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public double alliance;

  private boolean m_isOpenLoop;
  public boolean isCreepin = false;

  public PIDController skewApril_pid;
  public PIDController autoBalance_pid;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private ShuffleboardTab drivetrain_tab = Shuffleboard.getTab("Drivetrain");
  private Field2d field = new Field2d();

  public boolean isAuton = false;

  private SwerveModuleState[] m_states =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public SwerveDrivePoseEstimator poseEstimator;

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

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    ;
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
    // if (!isAuton) m_states = freezeLogic(m_states);

    setModuleStates();
    poseEstimator.update(getYaw(), getModulePositions());

    for (SwerveModule mod : mSwerveMods) {
      // if (Math.abs(mod.getCanCoder().getDegrees() - mod.getPosition().angle.getDegrees()) > 5) {
      //   // System.out.println("CANCoder Number: " + mod.moduleNumber);
      //   // System.out.println("CanCoder: " + mod.getCanCoder().getDegrees());
      //   // System.out.println("Integrated: " + mod.getPosition().angle.getDegrees());
      // }

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    updateNT();
  }

  private void updateNT() {
    ChassisSpeeds currentChassisSpeeds = getChassisSpeeds(true);
    Pose2d currentPose = getPose();

    SmartDashboard.putNumber("X Velocity", currentChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("X Velocity Setpoint", -m_chassisSpeeds.vxMetersPerSecond);

    SmartDashboard.putNumber("Y Velocity", currentChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Y Velocity Setpoint", -m_chassisSpeeds.vyMetersPerSecond);

    SmartDashboard.putNumber("Rotation Velocity", currentChassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Rotation Velocity Setpoint", m_chassisSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("/pose/th", getYaw().getRadians());
    SmartDashboard.putNumber("/pose/x", currentPose.getX());
    SmartDashboard.putNumber("/pose/y", currentPose.getY());

    SmartDashboard.putNumber("/poseEstimator/poseTH", getYaw().getRadians());
    SmartDashboard.putNumber("/poseEstimator/poseX", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("/poseEstimator/poseY", poseEstimator.getEstimatedPosition().getY());

    field.setRobotPose(
        poseEstimator.getEstimatedPosition().getX(),
        8.0137 + poseEstimator.getEstimatedPosition().getY(),
        getYaw());
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

  public void updatePoseLimelight(double[] pose, double latency) {
    Pose2d newPose = new Pose2d(pose[0], pose[1], getYaw());
    poseEstimator.resetPosition(getYaw(), getModulePositions(), newPose);

    System.out.println("ODOMETRY WAS RESET");
    System.out.println(newPose.toString());
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
    poseEstimator.resetPosition(newRot, getModulePositions(), newPose);
    ahrs.setAngleAdjustment(newRot.getDegrees());
  }

  public void zeroGyroscope() {
    ahrs.setAngleAdjustment(0);
    ahrs.reset();
    swerveOdometry.resetPosition(
        new Rotation2d(0), getModulePositions(), swerveOdometry.getPoseMeters());

    poseEstimator.resetPosition(
        new Rotation2d(0), getModulePositions(), poseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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

  public double getPitch() {
    return ahrs.getPitch();
  }

  public void yesCreepMode() {
    isCreepin = true;
  }

  public void noCreepMode() {
    isCreepin = false;
  }

  public void updateAlliance() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      alliance = 1;
    } else {
      alliance = 2;
    }
  }
}
