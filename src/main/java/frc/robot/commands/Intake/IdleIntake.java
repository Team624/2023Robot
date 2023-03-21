// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IdleIntake extends CommandBase {
  /** Creates a new IdleIntake. */
  private final Intake m_Intake;

  private final boolean m_cone;

  public IdleIntake(Intake intake, boolean cone) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Intake = intake;
    this.m_cone = cone;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_cone) {
      m_Intake.stopIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_cone) {
      m_Intake.runIntake(0.15);
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
