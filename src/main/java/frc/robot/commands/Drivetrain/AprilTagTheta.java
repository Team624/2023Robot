// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import java.util.function.DoubleSupplier;

public class AprilTagTheta extends CommandBase {
  /** Creates a new AprilTagTheta. */
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private SlewRateLimiter filterX = new SlewRateLimiter(4.5);
  private SlewRateLimiter filterY = new SlewRateLimiter(4.5);

  public AprilTagTheta(
      Drivetrain drivetrain,
      Limelight limelight,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double thVelocity = 0;

    if (m_limelight.hasTarget()) {
      double skew_angle = m_limelight.getAlignmentValues()[1];
      thVelocity = getSkewPID(skew_angle);

      if (skew_angle < 0) {
        thVelocity = -1;
      } else {
        thVelocity = 1;
      }
    }

    double xVelocity = m_translationXSupplier.getAsDouble();
    double yVelocity = m_translationYSupplier.getAsDouble();

    xVelocity = filterX.calculate(xVelocity);
    yVelocity = filterY.calculate(yVelocity);

    m_drivetrain.drive(new Translation2d(xVelocity, yVelocity), thVelocity, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, true, false);
    System.out.println("interrupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_limelight.hasTarget()) {
      System.out.println("angle: " + m_limelight.getAlignmentValues()[1]);
      if (Math.abs(m_limelight.getAlignmentValues()[1]) < 3) {
        return true;
      }
    }
    return false;
  }

  private double getSkewPID(double wantedDeltaAngle) {
    return m_drivetrain.skewApril_pid.calculate(
        m_drivetrain.getYaw().getDegrees(), m_drivetrain.getYaw().getDegrees() + wantedDeltaAngle);
  }
}
