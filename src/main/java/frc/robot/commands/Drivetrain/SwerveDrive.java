// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  Drivetrain m_drivetrain;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter filterX = new SlewRateLimiter(7);
  private SlewRateLimiter filterY = new SlewRateLimiter(7);

  public SwerveDrive(
      Drivetrain s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double translationVal =
    //     MathUtil.applyDeadband(
    //         translationSup.getAsDouble(), Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND);
    // double strafeVal =
    //     MathUtil.applyDeadband(strafeSup.getAsDouble(),
    // Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND);
    // double rotationVal =
    //     MathUtil.applyDeadband(
    //         rotationSup.getAsDouble(), Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND);

    // m_drivetrain.drive(
    //     new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
    //     -rotationVal * Constants.Swerve.maxAngularVelocity,
    //     !robotCentricSup.getAsBoolean(),
    //     true);

    double vx = translationSup.getAsDouble();
    double vy = strafeSup.getAsDouble();
    double omega = rotationSup.getAsDouble();

    vx *= Constants.Swerve.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
    vy *= Constants.Swerve.DRIVETRAIN_INPUT_TRANSLATION_MULTIPLIER;
    omega *= Constants.Swerve.DRIVETRAIN_INPUT_ROTATION_MULTIPLIER;

    // the creep mode
    // if (m_drivetrainSubsystem.isCreepin) {
    //     vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
    //     vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
    //     omega *= Constants.Drivetrain.DRIVETRAIN_INPUT_CREEP_MULTIPLIER;
    // } else if (m_drivetrainSubsystem.isSpeedin) {
    //     vx *= Constants.Drivetrain.DRIVETRAIN_INPUT_SPEED_MULTIPLIER;
    //     vy *= Constants.Drivetrain.DRIVETRAIN_INPUT_SPEED_MULTIPLIER;
    // }

    vx = filterX.calculate(vx);
    vy = filterY.calculate(vy);

    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, m_drivetrain.getYaw()));
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
