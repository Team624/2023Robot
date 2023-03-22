package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class DisabledLimelight extends CommandBase {
  private Limelight m_limelight;

  public DisabledLimelight(Limelight limelight) {
    this.m_limelight = limelight;
    this.addRequirements(limelight);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
