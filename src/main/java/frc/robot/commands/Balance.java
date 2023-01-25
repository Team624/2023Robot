package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final double angleThreshold = 6;
  private double angle;
  private PIDController pidController;

  public Balance(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    angle = 0;
    pidController = new PIDController(0.034, 0, 0);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    angle = m_drivetrain.getAngle();
    double pid_val = pidController.calculate(angle);
    System.out.println(angle + "° : " + pid_val);
    m_drivetrain.drive(new Translation2d(pidController.calculate(angle), 0), 0, true, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    
    if (-angleThreshold < angle && angle < angleThreshold){
      m_drivetrain.drive(new Translation2d(0, 0), 0.5, true, false);
      return true;
    }
    return false;
  }
}
