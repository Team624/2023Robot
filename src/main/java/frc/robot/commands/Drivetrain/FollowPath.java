// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Path;
import frc.robot.utility.PathPoint;

public class FollowPath extends CommandBase {
  private final Drivetrain drivetrain;

  private Path path;
  private int currentPointIndex = 0;
  private PathPoint point;
  private HolonomicDriveController controller;

  public FollowPath(Drivetrain drive, Path path) {
    this.drivetrain = drive;
    this.path = path;
    this.point = path.getPoint(currentPointIndex);

    this.controller = new HolonomicDriveController(
      new PIDController(
        Constants.Autonomous.DRIVE_CONTROLLER_X_KP,
        Constants.Autonomous.DRIVE_CONTROLLER_X_KP, 
        Constants.Autonomous.DRIVE_CONTROLLER_X_KP), 
      new PIDController(
        Constants.Autonomous.DRIVE_CONTROLLER_Y_KP, 
        Constants.Autonomous.DRIVE_CONTROLLER_Y_KP, 
        Constants.Autonomous.DRIVE_CONTROLLER_Y_KP), 
      new ProfiledPIDController(
        Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KP, 
        Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KI, 
        Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KD,
        new TrapezoidProfile.Constraints(
          Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY,
          Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_ACCELERATION
        )
      )
    );

    controller.setTolerance(new Pose2d(point.getTolerance(), point.getTolerance(), Constants.Autonomous.AUTONOMOUS_ROTATION_TOLERANCE));

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting path " + path.getPathId());
    updateNTPoint();
    updateNTFinishedPath(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.atReference() && !nextPoint() || currentPointIndex >= path.getLength() - 1) return;

    ChassisSpeeds chassisSpeeds = controller.calculate( 
      drivetrain.getPose(), 
      path.getPoint(currentPointIndex + 1).getPose(), 
      // Combine x and y velocities from ros into one linear velocity
      Math.hypot(point.getVx(), point.getVy()), 
      Rotation2d.fromRadians(path.getPoint(currentPointIndex + 1).getHeading())
    );
    
    drivetrain.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    updateNTFinishedPath(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentPointIndex + 1 >= path.getLength());
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }

  // Increments the current point if one exists.
  // Returns whether there was another point.
  private boolean nextPoint() {
    if (currentPointIndex + 1 >= path.getLength()) return false;
    
    currentPointIndex++;

    updateNTPoint();

    point = path.getPoint(currentPointIndex);

    System.out.println("Starting point " + currentPointIndex);

    return true;
  }

  // Updates NetworkTables with the current point for ROS.
  private void updateNTPoint() {
    SmartDashboard.getEntry("/pathTable/status/point").setNumber(currentPointIndex);
  }

  // Updates NetworkTables with the completion status of the path for ROS.
  private void updateNTFinishedPath(boolean finished) {
    SmartDashboard.getEntry("/pathTable/status/finishedPath")
        .setString(finished + " " + path.getPathId());
  }
}
