// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DeployIntake extends CommandBase {
  /** Creates a new DeployIntake. */
  private final Intake m_Intake;

  private final XboxController m_Controller;

  public DeployIntake(Intake intake, XboxController Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Intake = intake;
    this.m_Controller = Controller;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_Intake.deploySolenoids();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Intake.runIntake(1);
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
