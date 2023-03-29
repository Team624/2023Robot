package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ReflectiveAlign extends CommandBase {

  private final Drivetrain m_drivetrain;

  private Limelight limelight;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;

  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(0.001, 0.0, 0.0, Y_CONSTRAINTS);

  public ReflectiveAlign(Drivetrain m_drivetrain, Limelight limelight) {
    this.limelight = limelight;
    this.m_drivetrain = m_drivetrain;

    yController.setGoal(0);
  }

  public void execute() {
    double feedback = yController.calculate(limelight.getAngle());

    m_drivetrain.drive(new ChassisSpeeds(0, feedback, 0), true, false);
  }

  public void initialize() {
    yController.setTolerance(.01);
  }

  public boolean isFinished() {
    return yController.atGoal();
  }
}
