// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm3 extends CommandBase {
  /** Creates a new SetArm3. */
  private final Arm m_Arm;

  private final double m_givenSetpoint;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1.1, 0.75);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public SetArm3(Arm arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    this.m_givenSetpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpoint = new TrapezoidProfile.State(m_Arm.getBoreEncoder(), 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_goal = new TrapezoidProfile.State(m_givenSetpoint, 0);
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    m_setpoint = profile.calculate(0.02);

    m_Arm.ArmProfile(m_setpoint);
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
