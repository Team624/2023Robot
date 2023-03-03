// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends CommandBase {
  /** Creates a new ReverseIntake. */
  private final Intake m_intake;

  private final Arm m_Arm;

  public ReverseIntake(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.m_intake = intake;
    this.m_Arm = arm;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Arm.cone) {
      m_intake.runIntake(-0.6);
    } else {
      m_intake.runIntake(-0.3);
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
