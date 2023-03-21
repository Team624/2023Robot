// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class SetTelescopeScore extends CommandBase {
  /** Creates a new SetTelescopeScore. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final boolean m_cone;
  private final boolean m_score;
  private boolean running = false;

  public SetTelescopeScore(Arm arm, Telescope telescope, boolean cone, boolean score) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_cone = cone;
    this.m_score = score;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Arm.getAbsoluteRotation().getDegrees() < 180 && m_cone && m_score) {
      m_Telescope.setTelescope(0.15);
      running = true;
    }
    if (m_Arm.getAbsoluteRotation().getDegrees() > 180 && m_cone && !m_score) {
      m_Telescope.setTelescope(0.15);
      running = true;
    } else {
      running = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (running) {
      if (Math.abs(m_Telescope.getStringPot() - 0.15) < 0.01) {
        return true;
      }
    }
    return false;
  }
}
