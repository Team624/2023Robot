// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Path;

public class FollowPath extends CommandBase {
  private final Drivetrain drivetrain;
  private final Path path;

  private HolonomicDriveController controller;
  private Timer timer;

  public FollowPath(Drivetrain drive, Path path) {
    this.drivetrain = drive;
    this.path = path;

    this.controller =
        new HolonomicDriveController(
            new PIDController(
                Constants.Autonomous.DRIVE_CONTROLLER_X_KP,
                Constants.Autonomous.DRIVE_CONTROLLER_X_KI,
                Constants.Autonomous.DRIVE_CONTROLLER_X_KD),
            new PIDController(
                Constants.Autonomous.DRIVE_CONTROLLER_Y_KP,
                Constants.Autonomous.DRIVE_CONTROLLER_Y_KI,
                Constants.Autonomous.DRIVE_CONTROLLER_Y_KD),
            new ProfiledPIDController(
                Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KP,
                Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KI,
                Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KD,
                new TrapezoidProfile.Constraints(
                    Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_VELOCITY,
                    Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_MAX_ACCELERATION)));

    controller.setTolerance(
        new Pose2d(
            Constants.Autonomous.AUTONOMOUS_X_TOLERANCE,
            Constants.Autonomous.AUTONOMOUS_Y_TOLERANCE,
            Constants.Autonomous.AUTONOMOUS_ROTATION_TOLERANCE));

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting path " + path.getPathId());
    updateNTFinishedPath(false);

    timer = new Timer();

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timeSeconds = timer.get();

    Pose2d wantedPose = path.interpolate(timeSeconds);
    wantedPose =
        new Pose2d(
            wantedPose.getTranslation(),
            new Rotation2d(
                MathUtil.inputModulus(wantedPose.getRotation().getRadians(), 0.0, 2 * Math.PI)));

    Pose2d currentPose = drivetrain.getPose();
    currentPose =
        new Pose2d(
            currentPose.getTranslation(),
            new Rotation2d(
                MathUtil.inputModulus(drivetrain.getYaw().getRadians(), 0.0, 2 * Math.PI)));
    // currentPose = new Pose2d(currentPose.getTranslation(), new
    // Rotation2d(MathUtil.inputModulus(currentPose.getRotation().getRadians(), 0, 2 * Math.PI)));

    System.out.println("Wanted: " + wantedPose.getRotation().getRadians());

    System.out.println("Current: " + currentPose.getRotation().getRadians());

    SmartDashboard.putNumber(
        "Heading Error", wantedPose.getRotation().minus(currentPose.getRotation()).getRadians());

    SmartDashboard.putNumber("Wanted Heading", wantedPose.getRotation().getRadians());

    System.out.println(
        "Error: " + wantedPose.getRotation().minus(currentPose.getRotation()).getRadians());

    ChassisSpeeds chassisSpeeds =
        controller.calculate(
            currentPose, wantedPose, path.getVelocity(timeSeconds), wantedPose.getRotation());

    drivetrain.drive(chassisSpeeds, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    updateNTFinishedPath(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= path.getSeconds()); // && controller.atReference()
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }

  // Updates NetworkTables with the completion status of the path for ROS.
  private void updateNTFinishedPath(boolean finished) {
    SmartDashboard.getEntry("/pathTable/status/finishedPath")
        .setString(finished + " " + path.getPathId());
  }
}
