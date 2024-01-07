// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LockedRotationSwerveDrive extends CommandBase {
  Drivetrain m_drivetrain;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter filterX = new SlewRateLimiter(10);
  private SlewRateLimiter filterY = new SlewRateLimiter(10);

  private ProfiledPIDController thetaController =
      new ProfiledPIDController(
          Constants.Autonomous.DRIVE_LOCKED_ROTATION_KP,
          Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KI,
          Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KD,
          new TrapezoidProfile.Constraints(
              Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY,
              Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_ACCELERATION));

  public LockedRotationSwerveDrive(
      Drivetrain s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      double theta,
      BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();
    thetaController.setGoal(0.0);
    double rotationVal =
        thetaController.calculate(m_drivetrain.getPose().getRotation().getRadians());
    // rotationVal = MathUtil.clamp(rotationVal, -, 0.5);

    System.out.println(rotationVal);

    if (m_drivetrain.isCreepin) {
      translationVal *= 0.4;
      strafeVal *= 0.4;
    }

    translationVal = filterX.calculate(translationVal);
    strafeVal = filterY.calculate(strafeVal);

    m_drivetrain.drive(
        new Translation2d(translationVal, strafeVal)
            .times(Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
        rotationVal,
        !robotCentricSup.getAsBoolean(),
        true);
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
