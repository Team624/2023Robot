// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GoalPose extends CommandBase {
  /** Creates a new GoalPose. */
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  public static final double AngVel = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, 8);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  public GoalPose(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d pose = m_drivetrain.getPose();
    omegaController.reset(pose.getRotation().getRadians());
    xController.reset(pose.getX());
    yController.reset(pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] pose2d = m_drivetrain.getSwervePose();
    xController.setGoal(pose2d[0] + 1.1);
    yController.setGoal(pose2d[1] + 1.1);
    omegaController.setGoal(m_drivetrain.getYaw().getDegrees() + 15);

    System.out.println("monkey2" + pose2d[0]);
    if (m_limelight.hasTarget()) {
      double xVel = xController.calculate(pose2d[0]);
      System.out.println(xVel + "monkey");
      if (xController.atGoal()) {
        xVel = 0.0;
      }
      double yVel = yController.calculate(pose2d[1]);
      if (yController.atGoal()) {
        yVel = 0.0;
      }
      // double thVel = omegaController.calculate(m_drivetrain.getYaw().getDegrees());
      // if (omegaController.atGoal()) {
      //   thVel = 0.0;
      // }
      m_drivetrain.drive(new Translation2d(xVel, yVel), 0, true);
    }
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
