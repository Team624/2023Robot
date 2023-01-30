// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GoalPose extends CommandBase {
  /** Creates a new GoalPose. */
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;
  private final int m_node;
  private final int m_right;

  public double goal;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  public static final double AngVel = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, 3);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(2.5, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  public GoalPose(Drivetrain drivetrain, Limelight limelight, int node, int right) {
    // Use addRequirements() here to declare subsystem dependencies.

    // right 1
    // left 2

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_node = node;
    this.m_right = right;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
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

    // 0 = left: -3.6031665
    // 1 = middle: -5.25404
    // 2 = right: -6.98812

    // id_json.put(1.0, -2.93659);
    // id_json.put(2.0, -1.26019);
    // id_json.put(3.0, 0.41621);
    // id_json.put(4.0, 2.74161);
    // id_json.put(5.0, 2.74161);
    // id_json.put(6.0, 0.41621);
    // id_json.put(7.0, -1.26019);
    // id_json.put(8.0, -2.93659);

    // -4.01

    // Tag 1: -6.983
    // Tag 2 y value: -5.2995
    // Tag 3: 3.61357
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d pose2d = m_drivetrain.getPose();

    if (m_node == 0) {
      goal = -3.58;

    } else if (m_node == 1) {
      goal = -5.277685;

    } else if (m_node == 2) {
      goal = -6.987;
    } else {
      goal = m_limelight.getYofID();
    }

    if (m_right == 0) {
      yController.setGoal(goal - (22 / 39.37));
    } else if (m_right == 1) {
      yController.setGoal(goal + (22 / 39.37));
    } else {
      yController.setGoal(goal);
    }

    omegaController.setGoal(0);

    double yVel = yController.calculate(pose2d.getY());

    double thVel = omegaController.calculate(pose2d.getRotation().getRadians());

    m_drivetrain.drive(new Translation2d(0, yVel), thVel, true);
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("y controllers " + yController.getPositionError());
    if (yController.atGoal()) {

      return true;
    }

    return false;
  }
}
