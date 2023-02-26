// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWrist extends CommandBase {
  /** Creates a new setWrist. */
  private final Wrist m_Wrist;

  private final double m_setPoint;

  private static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, 1.1);

  private final ProfiledPIDController wristController =
      new ProfiledPIDController(0.0003, 0, 0, WRIST_CONSTRAINTS);

  public SetWrist(Wrist wrist, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Wrist = wrist;
    this.m_setPoint = setpoint;
    addRequirements(wrist);
    wristController.setTolerance(0.05);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristController.reset(m_Wrist.getBoreEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Rotation2d setPoint2d = new Rotation2d(m_setPoint);
    // m_Wrist.setWristCommand(m_setPoint, setPoint2d);

    wristController.setGoal(m_setPoint);

    double vel = wristController.calculate(m_Wrist.getBoreEncoder());

    m_Wrist.setWristCommand(vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
