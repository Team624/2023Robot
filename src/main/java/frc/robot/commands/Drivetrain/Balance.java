package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {

  private final Drivetrain m_drivetrain;
  private double angle;
  private PIDController pidController;

  public double goal;

  boolean ground;
  private final boolean m_front;

  public Balance(Drivetrain drivetrain, boolean front) {
    this.m_drivetrain = drivetrain;
    this.m_front = front;
    angle = 0;
    pidController = new PIDController(0.053, 0, 0);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    ground = true;
  }

  @Override
  public void execute() {
    // x component of charge station = 3.88745 meters away from alliance wall
    double vel = 1.8;
    double mult = 1;

    angle = m_drivetrain.getPitch();

    if (DriverStation.getAlliance() == Alliance.Red) {
      vel = -1.8;
      mult = -1;
    }

    if (m_front) {
      vel = -vel;
      mult *= -1;
    }

    if (Math.abs(angle) < 15) {
      m_drivetrain.drive(new Translation2d(vel, 0), 0, true, true);
    }

    if (Math.abs(angle) > 15) {
      ground = false;
    }

    if (!ground) {
      m_drivetrain.drive(
          new Translation2d(pidController.calculate(-Math.abs(angle) * mult), 0), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    System.out.println("the angle: " + angle);
    if (Math.abs(angle) < 13.8 && !ground) {
      m_drivetrain.drive(new Translation2d(0, 0), 0.5, true, true);

      // m_drivetrain.swerveXposition();
      setNTState(true);

      return true;
    }
    return false;
  }

  private void setNTState(boolean state) {
    SmartDashboard.getEntry("/auto/balance/state").setBoolean(state);
  }
}
