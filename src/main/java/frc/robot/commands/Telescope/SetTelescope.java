// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class SetTelescope extends CommandBase {
  /** Creates a new SetTelescope. */
  private final Telescope m_Telescope;

  private final double m_setPoint;

  public SetTelescope(Telescope telescope, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Telescope = telescope;
    this.m_setPoint = setPoint;
    addRequirements(telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Telescope.setTelescope(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Telescope.getEncoder() - 2 < 0.1;
  }
}
