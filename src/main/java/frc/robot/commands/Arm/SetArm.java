// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  private final Arm m_Arm;

  private final double m_Setpoint;

  private static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
      new TrapezoidProfile.Constraints(0.25, 0.3);

  private final ProfiledPIDController armController =
      new ProfiledPIDController(10, 0, 0, ARM_CONSTRAINTS);

  public SetArm(Arm arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    this.m_Setpoint = setpoint;
    armController.setTolerance(0.01);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.reset(m_Arm.getBoreEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    double vel = armController.calculate(m_Arm.getBoreEncoder(),0.8);
    System.out.println("voltage "+vel);

    m_Arm.setArmCommand(vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("GOAL: " + armController.getSetpoint().position);
    // System.out.println("BORE: " + m_Arm.getBoreEncoder());
    // System.out.println("offset: " + armController.getPositionError());
    return armController.atGoal();
  }
}
