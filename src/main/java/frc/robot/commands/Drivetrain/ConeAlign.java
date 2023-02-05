package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ConeAlign extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;
  private final double distance = 22 / 39.37;

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
      new ProfiledPIDController(2, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  public ConeAlign(Drivetrain drivetrain, Limelight limelight) {

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    omegaController.setTolerance(Units.degreesToRadians(3));
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
    System.out.println("Currect: " + m_drivetrain.getPose().getY());
    double yVel = 0;
    Pose2d pose2d = m_drivetrain.getPose();
    if (m_limelight.getYofTag() != 0) {
      goal = m_limelight.getYofTag() + distance;
      System.out.println(goal);
      yController.setGoal(goal);
      yVel = yController.calculate(pose2d.getY());
    } else {
      yController.setGoal(m_drivetrain.getPose().getY());
    }

    omegaController.setGoal(0);

    m_drivetrain.drive(new Translation2d(0, yVel), 0, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    System.out.println("Position Error " + yController.getPositionError());
    return yController.atGoal();
  }
}
