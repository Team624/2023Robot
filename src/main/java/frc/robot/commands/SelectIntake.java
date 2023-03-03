// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

public class SelectIntake extends CommandBase {
  /** Creates a new RunUprightConeIntake. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;

  public SelectIntake(Arm arm, Telescope telescope, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_Arm.cone) {
      new UprightConeIntake(m_Arm, m_Telescope, m_Wrist);
    } else {
      new IntakeSequence(m_Arm, m_Telescope, m_Wrist);
    }
  }

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
