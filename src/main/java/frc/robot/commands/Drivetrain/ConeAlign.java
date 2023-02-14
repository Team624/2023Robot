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
      new ProfiledPIDController(1.8, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);

  private final boolean m_right;

  public ConeAlign(Drivetrain drivetrain, Limelight limelight, Boolean right) {

    m_drivetrain = drivetrain;
    m_limelight = limelight;
    this.m_right = right;
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    Pose2d pose = m_drivetrain.getPose();

    omegaController.reset(pose.getRotation().getRadians());

    xController.reset(pose.getX());
    yController.reset(pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yVel = 0;
    double rotSpeed = 0;
    if (m_limelight.getYofTag() != 0) {
      if (m_right) {
        goal = m_limelight.getYofTag() - distance;
      } else {
        goal = m_limelight.getYofTag() + distance;
      }

      yController.setGoal(goal);
      System.out.println("GOAL: " + goal);
      double angle = m_limelight.alignment_values()[1];
      System.out.println(m_drivetrain.getPose().getRotation().getDegrees());
      omegaController.setGoal(-Math.PI);
      rotSpeed = angle > 0 ? 1 : -1;
      yVel = yController.calculate(m_drivetrain.getPose().getY());
      rotSpeed = omegaController.calculate(m_drivetrain.getPose().getRotation().getRadians());
    }
    System.out.println("Y VEL: " + yVel);
    m_drivetrain.drive(new Translation2d(0, yVel), rotSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getPose().getY() - goal) < .01 && omegaController.atGoal();
  }
}
