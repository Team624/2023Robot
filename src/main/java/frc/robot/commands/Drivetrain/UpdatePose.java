// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class UpdatePose extends CommandBase {
  /** Creates a new UpdatePose. */
  private final Limelight m_Limelight;

  private final Drivetrain m_drivetrain;

  public UpdatePose(Drivetrain Drivetrain, Limelight Limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = Drivetrain;
    this.m_Limelight = Limelight;

    addRequirements(Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.updatePoseLimelight(m_Limelight.getBotPose());
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
