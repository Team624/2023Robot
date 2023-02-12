// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

public class ToHighGoal extends CommandBase {
  private final Arm arm;
  private final Pivot pivot;
  /** Creates a new ToHighGoal. */
  public ToHighGoal(Arm arm, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.pivotTo(59);
    arm.extendTo(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((pivot.getEncoder() <= 63 && pivot.getEncoder() > 55)
        && (arm.getEncoder() < 5 && arm.getEncoder() > -5));
  }
}
