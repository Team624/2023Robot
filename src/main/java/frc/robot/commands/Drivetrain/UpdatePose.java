// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class UpdatePose extends CommandBase {
  /** Creates a new UpdatePose. */
  private final Limelight m_Limelight;

  private final Drivetrain m_Drivetrain;

  public UpdatePose(Limelight Limelight, Drivetrain drivetrian) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Limelight = Limelight;
    this.m_Drivetrain = drivetrian;

    addRequirements(Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Limelight.hasTarget()) {
      Translation2d newTranslation =
          new Translation2d(m_Limelight.getBotPose()[0], m_Limelight.getBotPose()[1]);
      Rotation2d newRotation = new Rotation2d(m_Limelight.getBotPoseAngle());
      Pose2d newPose2d = new Pose2d(newTranslation, newRotation);
      m_Drivetrain.updatePoseLimelight(newPose2d);
    }
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
