package frc.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Manages the auton selection box in Shuffleboard
public class AutonSelection extends CommandBase {
  private ShuffleboardTab autonTab;
  private GenericEntry shouldBalanceGet;
  private SendableChooser<Integer> chooser;

  @Override
  public void initialize() {
    this.autonTab = Shuffleboard.getTab("Autonomous");

    this.shouldBalanceGet = 
      autonTab
        .add("Balance?", true)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();

    this.chooser = new SendableChooser<Integer>();

    this.chooser.setDefaultOption("Do Nothing", 0);

    autonTab.add("Choose Auton", chooser).withSize(2, 1).withPosition(0, 0);

    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");

    System.out.println("Started disabled command");
  }


  @Override
  public void execute() {
    boolean shouldBalance = shouldBalanceGet.getBoolean(true);

    SmartDashboard.putBoolean("/auto/balance/should_balance", shouldBalance);

    int numAutons = (int) SmartDashboard.getNumber("/auto/num_autons", 0);

    for (int i = 1; i <= numAutons; i++) {
      String name = SmartDashboard.getString("/auto/autons/auton" + i, "N/A");
      this.chooser.addOption(name, i);
    }

    Integer choice = chooser.getSelected();

    if (choice != null) {
      SmartDashboard.putNumber("/auto/select", choice);
    }
    
    SmartDashboard.putBoolean("/auto/is_blue", DriverStation.getAlliance() == Alliance.Blue);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
