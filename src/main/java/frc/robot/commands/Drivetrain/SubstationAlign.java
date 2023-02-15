package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SubstationAlign extends CommandBase {

  private boolean red_alliance;

  private final Drivetrain m_drivetrain;

  private double distance = 90.77 / 39.37 - .58 / 2;

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
      new ProfiledPIDController(3, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);

  public SubstationAlign(Drivetrain drivetrain, Boolean red_alliance) {
    this.red_alliance = red_alliance;
    m_drivetrain = drivetrain;
    this.red_alliance = red_alliance;
    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d pose = m_drivetrain.getPose();

    omegaController.reset(MathUtil.angleModulus(pose.getRotation().getRadians()));

    xController.reset(pose.getX());
    yController.reset(pose.getY());
  }

  @Override
  public void execute() {
    if (red_alliance) {
      System.out.println("X goal: " + distance);
      xController.setGoal(distance);
    } else {
      System.out.println("X goal: " + (16.54 - distance));
      xController.setGoal(16.54 - distance);
    }
    omegaController.setGoal(-Math.PI / 2);
    yController.setGoal(-1.26839);
    double yVel = yController.calculate(m_drivetrain.getPose().getY());
    double xVel = xController.calculate(m_drivetrain.getPose().getX());
    double rotSpeed =
        omegaController.calculate(
            MathUtil.angleModulus(m_drivetrain.getPose().getRotation().getRadians()));
    m_drivetrain.drive(new Translation2d(xVel, yVel), rotSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return yController.atGoal() && xController.atGoal() && omegaController.atGoal();
  }
}
