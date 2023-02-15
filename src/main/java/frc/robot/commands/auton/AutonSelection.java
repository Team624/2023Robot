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
  private GenericEntry autonNameWidget;
  private GenericEntry shouldBalanceGet;

  @Override
  public void initialize() {
    this.autonTab = Shuffleboard.getTab("Autonomous");
    this.autonChoiceGet =
        autonTab
            .add("Auton ID", 0)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    this.autonNameWidget =
        autonTab
            .add("Name", "N/A")
            .withPosition(0, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    this.shouldBalanceGet = 
      autonTab
        .add("Balance?", true)
        .withPosition(1, 0)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();

    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");

    System.out.println("Started disabled command");
  }


  @Override
  public void execute() {
    double autonChoice = autonChoiceGet.getDouble(0.0);
    boolean shouldBalance = shouldBalanceGet.getBoolean(true);

    SmartDashboard.putNumber("/auto/select", autonChoice);
    SmartDashboard.putBoolean("/auto/balance/should_balance", shouldBalance);

    String autonName = SmartDashboard.getString("/pathTable/auton_name", "N/A");

    autonNameWidget.setString(autonName);
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
