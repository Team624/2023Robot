// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class SetHoodIntake extends CommandBase {
  /** Creates a new SetHoodIntake. */
  private final Hood m_hood;

  public SetHoodIntake(Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hood.setGoal(Constants.Hood.Hood_Intake_Setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hood.setGoal(Constants.Hood.Hood_Upright_Setpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hood.getController().atGoal();
  }
}
