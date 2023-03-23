package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ConeAlign extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;

  public double goal;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  public static final double AngVel = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, 3);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(10.3, 0.15, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(9.5, 0, 0, OMEGA_CONSTRAINTS);

  private final boolean m_right;

  public ConeAlign(Drivetrain drivetrain, Boolean right, Limelight mLimelight) {

    m_drivetrain = drivetrain;
    m_limelight = mLimelight;
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      m_right = !right;
    } else {
      m_right = right;
    }
    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    omegaController.setTolerance(Units.degreesToRadians(2));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = m_drivetrain.getPose();
    omegaController.reset(pose.getRotation().getRadians());
    xController.reset(pose.getX());
    yController.reset(pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentY = m_drivetrain.getPose().getY();
    double[] possibleLocations = {
      -3.0349888823977644,
      -4.152591117602235,
      -4.711388882397765,
      -5.828991117602236,
      -6.387788882397764,
      -7.505391117602235
    };
    if (currentY > possibleLocations[0]) {
      goal = possibleLocations[0];
    } else if (currentY < possibleLocations[5]) {
      goal = possibleLocations[5];
    } else {
      for (int start = 0; start < 5; start++) {
        if (currentY <= possibleLocations[start] && currentY >= possibleLocations[start + 1]) {
          if (m_right) {
            goal = possibleLocations[start + 1];
          } else {
            goal = possibleLocations[start];
          }
          break;
        }
      }
    }
    yController.setGoal(goal);
    omegaController.setGoal(0.0);
    double yVel = yController.calculate(m_drivetrain.getPose().getY());
    double rotSpeed = omegaController.calculate(m_drivetrain.getPose().getRotation().getRadians());
    UpdatePose.keepRunning = false;
    m_drivetrain.drive(new Translation2d(0, yVel), rotSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    UpdatePose.keepRunning = true;
  }

  @Override
  public boolean isFinished() {
    return yController.atGoal() && omegaController.atGoal();
  }
}
