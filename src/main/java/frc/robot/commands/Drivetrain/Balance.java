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
    pidController = new PIDController(0.064, 0, 0);

    // P = 0.053
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

    if (m_front) {
      vel = -vel;
      mult *= -1;
    }

    if (Math.abs(angle) < 9) { // RSC 9
      m_drivetrain.drive(new Translation2d(vel, 0), 0, true, true);
    }

    // Comp 15

    if (Math.abs(angle) > 9) { // RSC 9
      ground = false;
    }

    // comp 15

    if (!ground) {
      m_drivetrain.drive(
          new Translation2d(pidController.calculate(-Math.abs(angle) * mult), 0), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // RSC 7
    // Comp 13.8

    System.out.println("the angle: " + angle);
    if (Math.abs(angle) < 8.5
        && !ground
        && m_drivetrain.getAngle() < 3
        && m_drivetrain.getAngle() > -3) {
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
