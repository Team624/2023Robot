// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class TelescopeHigh extends CommandBase {
  /** Creates a new TelescopeHigh. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  public TelescopeHigh(Arm arm, Telescope telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    this.m_Telescope = telescope;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Arm.getAbsoluteRotation().getDegrees() > (Constants.Arm.ARM_SETPOINT_PREHIGH_SCORE).getDegrees()){
      m_Telescope.setTelescope(Constants.Telescope.TELESCOPE_SETPOINT_HIGH);
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
  return Math.abs(m_Telescope.getStringPot() - Constants.Telescope.TELESCOPE_SETPOINT_HIGH) < 0.01;
  }
}
