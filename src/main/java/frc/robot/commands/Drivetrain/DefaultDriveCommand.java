// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  private Drivetrain m_drivetrain;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  public DefaultDriveCommand(
      Drivetrain m_drivetrain,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);

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

    double vx = translationSup.getAsDouble();
    double vy = strafeSup.getAsDouble();
    double omega = rotationSup.getAsDouble();

    vx *= Constants.Swerve.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
    vy *= Constants.Swerve.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
    omega *= Constants.Swerve.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER;

    System.out.println(omega);

    m_drivetrain.drive(
        new Translation2d(vx, vy).times(Constants.Swerve.maxSpeed), -omega, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
