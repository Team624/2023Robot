// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GoalPose extends CommandBase {
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;
  private final int m_node;
  private final int m_right;

  public double goal;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  public static final double AngVel = 2 * Math.PI;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, Math.pow(AngVel, 2));

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          Constants.Limelight.kTranslationP + 1.6,
          Constants.Limelight.kTranslationI + .2,
          Constants.Limelight.kTranslationD,
          Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(Constants.Limelight.kRotationP, 0, 0, OMEGA_CONSTRAINTS);

  public GoalPose(Drivetrain drivetrain, Limelight limelight, int node, int right) {

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_node = node;
    this.m_right = right;

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d pose = m_drivetrain.getPose();

    omegaController.reset((pose.getRotation().getRadians()));

    xController.reset(pose.getX());
    yController.reset(pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thVel = 0.0;

    Pose2d pose2d = m_drivetrain.getPose();

    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (m_node == 0) {
        goal = 0.41621 - 4.01;

      } else if (m_node == 1) {
        goal = -1.26019 - 4.01;

      } else if (m_node == 2) {
        goal = -2.93659 - 4.01;
      } else {
        goal = m_limelight.getYofTag();
      }

      if (m_right == 0) {
        yController.setGoal(goal + (22 / 39.37));
      } else if (m_right == 1) {
        yController.setGoal(goal - (22 / 39.37));
      } else {
        yController.setGoal(goal);
      }
    } else {
      if (m_node == 0) {
        goal = -2.93659 - 4.01;
      } else if (m_node == 1) {
        goal = -1.26019 - 4.01;

      } else if (m_node == 2) {
        goal = 0.41621 - 4.01;
      } else {
        goal = m_limelight.getYofTag();
      }

      if (m_right == 0) {
        yController.setGoal(goal - (22 / 39.37));
      } else if (m_right == 1) {
        yController.setGoal(goal + (22 / 39.37));
      } else {
        yController.setGoal(goal);
      }
    }

    omegaController.setGoal(0.0);

    double yVel = yController.calculate(pose2d.getY());

    if (m_limelight.getData().tv) {
      thVel = omegaController.calculate(m_drivetrain.getPose().getRotation().getRadians());
    }
    thVel = omegaController.calculate((m_drivetrain.getPose().getRotation().getRadians()));
    UpdatePose.keepRunning = false;
    m_drivetrain.drive(new Translation2d(0, yVel), thVel, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    UpdatePose.keepRunning = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atGoal() && omegaController.atGoal();
  }
}
