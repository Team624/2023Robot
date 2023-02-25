// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  Drivetrain m_drivetrain;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private SlewRateLimiter filterX = new SlewRateLimiter(10);
  private SlewRateLimiter filterY = new SlewRateLimiter(10);

  public SwerveDrive(
      Drivetrain s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();
    double rotationVal = rotationSup.getAsDouble();

    if (m_drivetrain.isCreepin) {
      translationVal *= 0.4;
      strafeVal *= 0.4;
      rotationVal *= 0.4;
    }

    translationVal = filterX.calculate(translationVal);
    strafeVal = filterY.calculate(strafeVal);

    m_drivetrain.drive(
        new Translation2d(translationVal, strafeVal)
            .times(Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
        rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
