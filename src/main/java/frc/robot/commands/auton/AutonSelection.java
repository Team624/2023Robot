package frc.robot.commands.auton;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Manages the auton selection box in Shuffleboard
public class AutonSelection extends CommandBase {
  private ShuffleboardTab autonTab;
  private GenericEntry autonChoiceGet;

  @Override
  public void initialize() {
    this.autonTab = Shuffleboard.getTab("Autonomous");
    this.autonChoiceGet =
        autonTab
            .add("Auton ID", 0)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    SmartDashboard.getEntry("/pathTable/status/finishedPath")
            .setString("false -1");

    System.out.println("Started disabled command");
  }

  @Override
  public void execute() {
    double autonChoice = autonChoiceGet.getDouble(0.0);
    SmartDashboard.putNumber("/auto/select", autonChoice);
  }

  @Override
  public void end(boolean interrupted) {
    autonChoiceGet.close();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
