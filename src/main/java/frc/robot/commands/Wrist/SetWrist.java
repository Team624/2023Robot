// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWrist extends CommandBase {
  /** Creates a new SetWrist. */
  private final Wrist m_wrist2;

  private final Rotation2d m_setPoint;

  public SetWrist(Wrist wrist2, Rotation2d setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_wrist2 = wrist2;
    this.m_setPoint = setpoint;
    addRequirements(m_wrist2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist2.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist2.setGoal(m_setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist2.disable();
    m_wrist2.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist2.getController().atGoal();
  }
}
