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

  private double distance = 2.68 - .58 / 2;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND / 2;
  public static final double AngVel = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, 3);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(6, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(6, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);

  public SubstationAlign(Drivetrain drivetrain, boolean red_alliance) {
    this.red_alliance = red_alliance;
    m_drivetrain = drivetrain;
    xController.setTolerance(0.05);
    yController.setTolerance(0.03);
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
      xController.setGoal(distance);
      omegaController.setGoal(Math.PI / 2);
    } else {
      xController.setGoal(16.54 - distance);
      omegaController.setGoal(-Math.PI / 2);
    }

    yController.setGoal(-1.26839);
    double yVel = yController.calculate(m_drivetrain.getPose().getY());
    double xVel = xController.calculate(m_drivetrain.getPose().getX());
    double rotSpeed =
        omegaController.calculate(
            MathUtil.angleModulus(m_drivetrain.getPose().getRotation().getRadians()));
    UpdatePose.keepRunning = false;
    m_drivetrain.drive(new Translation2d(xVel, yVel), rotSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    UpdatePose.keepRunning = true;
  }

  @Override
  public boolean isFinished() {
    return yController.atGoal() && xController.atGoal() && omegaController.atGoal();
  }
}
