package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {

  private final Drivetrain m_drivetrain;
  private double angle;
  private PIDController pidController;

  public double goal;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  public static final double AngVel = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);

  boolean ground;

  public Balance(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    angle = 0;
    pidController = new PIDController(0.066, 0, 0);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    ground = true;
  }

  @Override
  public void execute() {
    // x component of charge station = 3.88745 meters away from alliance wall

    angle = m_drivetrain.getPitch();
    double pid_val = pidController.calculate(angle);
    System.out.println(angle + "° : " + pid_val);

    if (Math.abs(angle) < 9) {
      m_drivetrain.drive(new Translation2d(2.0, 0), 0, true, true);
    }

    if (Math.abs(angle) > 9) {
      ground = false;
    }

    if (!ground) {
      m_drivetrain.drive(new Translation2d(pidController.calculate(angle), 0), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    if (Math.abs(angle) < 9 && !ground) {
      m_drivetrain.drive(new Translation2d(0, 0), 0.5, true, true);

      return true;
    }
    return false;
  }
}
