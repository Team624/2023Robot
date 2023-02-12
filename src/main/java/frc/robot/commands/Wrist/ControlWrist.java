// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class ControlWrist extends CommandBase {
  /** Creates a new ControlWrist. */
  private final Wrist m_Wrist;

  private final XboxController controller;

  public ControlWrist(Wrist wrist, XboxController mController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Wrist = wrist;
    this.controller = mController;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(controller.getRightY()) > 0.05) {
      m_Wrist.moveWrist(controller.getRightY());
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
