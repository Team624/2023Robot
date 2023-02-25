// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class setWrist extends CommandBase {
  /** Creates a new setWrist. */
  private final Wrist m_Wrist;

  private final double m_setPoint;

  public setWrist(Wrist wrist, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Wrist = wrist;
    this.m_setPoint = setpoint;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d setPoint2d = new Rotation2d(m_setPoint);
    m_Wrist.setWristCommand(m_setPoint, setPoint2d);
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
