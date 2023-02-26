// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Telescope;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class ControlTelescope extends CommandBase {
  /** Creates a new ControlTelescope. */
  private final Telescope m_Telescope;

  private final XboxController m_Controller;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Telescope.kS, Constants.Telescope.kV, Constants.Telescope.kA);

  public ControlTelescope(Telescope telescope, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_Telescope = telescope;
    this.m_Controller = controller;
    addRequirements(telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderValue = m_Telescope.getTelescopeEncoder();

    // if(encoderValue>=0.01 && encoderValue<=35){
    //   m_Telescope.controlTelescope(-m_Controller.getRightY());
    // }
    // if(encoderValue<0.01){
    //   if(m_Controller.getRightY()<0){
    //     m_Telescope.controlTelescope(-m_Controller.getRightY());
    //   }

    // }
    // if(encoderValue>35){
    //   if(m_Controller.getRightY()>0){
    //     m_Telescope.controlTelescope(-m_Controller.getRightY());
    //   }
    // }

    m_Telescope.controlTelescope(-m_Controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
