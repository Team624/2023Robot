// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class SetIntakeArm extends CommandBase {
  /** Creates a new SetIntakeArm. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  public SetIntakeArm(Arm arm, Telescope telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm=arm;
    this.m_Telescope=telescope;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("im here");

    if(m_Telescope.getStringPot()> Constants.Telescope.TELESCOPE_SETPOINT_SIDE_CONE_INTAKE/2){
      m_Arm.setGoal(Constants.Arm.ARM_SETPOINT_SIDE_CONE_INTAKE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.disable();
    m_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.getController().atGoal()) {
      // System.out.println("At setpoint");
    }
    return m_Arm.getController().atGoal();
  }
}
