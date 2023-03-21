package frc.robot.commands.Telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class ResetEncoder extends CommandBase {
  private Telescope telescope;

  public ResetEncoder(Telescope telescope) {
    this.telescope = telescope;
  }

  @Override
  public void initialize() {
    this.telescope.resetEncoder();
    System.out.println("Resetting");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
