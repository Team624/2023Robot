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

public class VisionAprilTags extends CommandBase {
  /** Creates a new VisionApirlTags. */
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationThSupplier;

  private SlewRateLimiter filterX = new SlewRateLimiter(4.5);
  private SlewRateLimiter filterTh = new SlewRateLimiter(4.5);

  public VisionAprilTags(
      Drivetrain drivetrain,
      Limelight limelight,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationThSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationThSupplier = translationThSupplier;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double thVelocity = 0;
    double yVelocity = 0;

    double skew_angle = m_limelight.alignment_values()[1];

    thVelocity = getSkewPID(skew_angle);

    if (skew_angle < 0) {
      thVelocity = -0.5;
    } else {
      thVelocity = 0.5;
    }

    double horiz_distance = m_limelight.alignment_values()[0];

    double xVelocity = m_translationXSupplier.getAsDouble();

    if (horiz_distance < 0) {
      yVelocity = -0.4;
    } else {
      yVelocity = 0.4;
    }

    xVelocity = filterX.calculate(xVelocity);

    m_drivetrain.drive(new Translation2d(xVelocity, yVelocity), thVelocity, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, true, true);
    System.out.println("interrupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("distance: " + m_limelight.alignment_values()[0]);
    System.out.println("angle: " + m_limelight.alignment_values()[1]);

    if (Math.abs(m_limelight.alignment_values()[0]) < 0.1
        && Math.abs(m_limelight.alignment_values()[1]) < 5) {
      return true;
    }

    return false;
  }

  private double getSkewPID(double wantedDeltaAngle) {
    return m_drivetrain.skewApril_pid.calculate(
        m_drivetrain.getYaw().getDegrees(), m_drivetrain.getYaw().getDegrees() + wantedDeltaAngle);
  }
}
