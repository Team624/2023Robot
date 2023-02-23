// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TrapProfile extends CommandBase {

  /** Creates a new TrapProfile. */
  private final Arm m_Arm;

  private final double m_Setpoint;

  private static double kDt = 0.02;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
  private TrapezoidProfile.State m_Profilegoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_Profilesetpoint = new TrapezoidProfile.State();

  public TrapProfile(Arm arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_Arm = arm;
    this.m_Setpoint = setpoint;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Profilegoal = new TrapezoidProfile.State(m_Setpoint, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Profilesetpoint = new TrapezoidProfile.State(m_Arm.getBoreEncoder(), 0);

    var profile = new TrapezoidProfile(m_constraints, m_Profilegoal, m_Profilesetpoint);
    m_Profilesetpoint = profile.calculate(kDt);
    m_Arm.ArmProfile(m_Profilesetpoint);
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
