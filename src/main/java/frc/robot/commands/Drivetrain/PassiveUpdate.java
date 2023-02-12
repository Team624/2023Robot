// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class PassiveUpdate extends CommandBase {
  /** Creates a new PassiveUpdate. */
  private final Drivetrain m_Drivetrain;

  private final Limelight m_Limelight;

  public PassiveUpdate(Drivetrain drivetrian, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain = drivetrian;
    this.m_Limelight = limelight;

    addRequirements(drivetrian);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
