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

  public static final double MAX_VOLTAGE = 12.0;

  public SwerveDriveOdometry m_swerveOdometry;
  public SwerveModule[] mSwerveMods;
  private ShuffleboardTab drivetraintab = Shuffleboard.getTab("Drivetrain");
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] lstates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public boolean isAuton = false;
  public boolean isUsingVision = false;

  public boolean lastPointCommand = false;
  public boolean stopAuton = false;
  public PIDController autonPoint_pidPathRotation;

  private AHRS ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public Drivetrain() {
    m_frontLeftModule = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    m_frontRightModule = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    m_backLeftModule = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    m_backRightModule = new SwerveModule(3, Constants.Swerve.Mod3.constants);

    mSwerveMods =
        new SwerveModule[] {
          m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule
        };

    m_swerveOdometry =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, ahrs.getRotation2d(), getModulePositions());
  }

  @Override
  public void periodic() {
    m_swerveOdometry.update(getGyroscopeRotation(), getModulePositions());
    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveModulePosition[] positions = getModulePositions();

    states = freezeLogic(states);

    // for (SwerveModule mod : mSwerveMods) {
    //   drivetraintab.add("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //   drivetraintab.add(
    //       "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    //   drivetraintab.add(
    //       "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    // }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeftModule.set(
        states[0].speedMetersPerSecond
            / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond
            / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * MAX_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond
            / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * MAX_VOLTAGE,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond
            / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * MAX_VOLTAGE,
        states[3].angle.getRadians());

    if (isAuton) {
      m_swerveOdometry.update(getGyroscopeRotation(), positions);
    } else {
      m_swerveOdometry.update(getGyroscopeRotation(), getModulePositions());
    }

    updateROSpose();
  }

  private SwerveModuleState getState(SwerveModule module) {
    return new SwerveModuleState(module.getVelocity(), module.getAngle());
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

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, ahrs.getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setChassis(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  public double[] getSwervePose() {
    double[] pose = {
      m_swerveOdometry.getPoseMeters().getX(), m_swerveOdometry.getPoseMeters().getY()
    };
    return pose;
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveOdometry.resetPosition(ahrs.getRotation2d(), getModulePositions(), pose);
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

  public void zeroGyro() {
    ahrs.setAngleAdjustment(0.0);
    ahrs.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(ahrs.getAngle());
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

  public void setAuton(boolean state) {
    isAuton = state;
  }

  public void setPose() {
    zeroGyro();
    double[] zeros = {0.0, 0.0, 0.0};
    double[] startPosition = SmartDashboard.getEntry("/pathTable/startPose").getDoubleArray(zeros);
    Rotation2d newRot = new Rotation2d(-startPosition[2]);
    Pose2d newPose = new Pose2d(startPosition[0], startPosition[1], newRot);
    // SwerveModulePosition[] newPosition =
    m_swerveOdometry.resetPosition(newRot, getModulePositions(), newPose);
    ahrs.setAngleAdjustment(newRot.getDegrees());
  }

  public void updateROSpose() {
    SmartDashboard.putNumber("/pose/th", getGyroscopeRotation().getRadians());
    SmartDashboard.putNumber("/pose/x", m_swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("/pose/y", m_swerveOdometry.getPoseMeters().getY());
  }
}
