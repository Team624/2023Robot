// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterScore extends CommandBase {
  /** Creates a new ShooterScore. */
  private final Shooter m_shooter;

  private final double m_speed;
  private Timer timer;

  public ShooterScore(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooter;
    this.m_speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < .3) {
      m_shooter.setPercentOutput(0.2);
    } else {
      m_shooter.setPercentOutput(m_speed + m_shooter.addedPercentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 0.6;
  }
}
