// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.spi.CurrencyNameProvider;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.Balance;
import frc.robot.commands.Drivetrain.FollowPath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.BezierCurve;
import frc.robot.utility.Path;

// Gets states from NetworkTables (published by ros) during auton
public class AutonManager extends CommandBase {
  private Drivetrain drivetrain;
  private Path[] paths;
  private Command currentFollowPathCommand;
  private int previousPath = -1;
  private Command currentBalanceCommand;

  public AutonManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updatePaths();
    drivetrain.setPose();
    drivetrain.setAuton(true);
    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");

    SmartDashboard.putBoolean("/auto/state", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stop the drivetrain if a new path was not started
    startNTPath();
    startNTBalance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setAuton(false);
    currentFollowPathCommand.cancel();

    SmartDashboard.putBoolean("/auto/state", false);

    System.out.println("Auton ended!");
  }

  private boolean startNTBalance() {
    if (currentBalanceCommand != null) return false;

    boolean startBalance = SmartDashboard.getEntry("/auto/balance/set").getBoolean(false);

    if (startBalance) {
      currentFollowPathCommand.end(true);
      
      currentBalanceCommand = new Balance(drivetrain);
      currentBalanceCommand.schedule();
      
      System.out.println("Starting balance!!!");
    }

    return startBalance;
  }

  // Starts the path specified by ROS in NetworkTables
  // Returns whether a path was started
  private boolean startNTPath() {
    if (currentFollowPathCommand != null && currentFollowPathCommand.isScheduled()) return false;

    Number[] indexes =
        SmartDashboard.getEntry("/pathTable/startPathIndex").getNumberArray(new Number[0]);

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    for (Number index : indexes) {
      int i = (int) index.doubleValue();

      System.out.println("Rx " + i);

      if (i < 0 || i >= paths.length || i <= previousPath) return false;

      System.out.println("Starting " + i);

      commandGroup.addCommands(new FollowPath(this.drivetrain, this.paths[i]));

      previousPath = i;
    }

    currentFollowPathCommand = commandGroup;

    commandGroup.schedule();

    SmartDashboard.getEntry("/pathTable/startPathIndex").setNumberArray(new Number[0]);

    return true;
  }

  // Grabs all paths from NetworkTables and stores it in this.paths
  private void updatePaths() {
    int numPaths = SmartDashboard.getEntry("/pathTable/num_paths").getNumber(0).intValue();

    this.paths = new Path[numPaths];

    for (int i = 0; i < numPaths; i++) {
      this.paths[i] = getPath(i);
    }
  }

  // Grabs a path of specified index from NetworkTables
  private Path getPath(int pathIndex) {
    String pathRoot = "/pathTable/path" + pathIndex;

    double timeSeconds = SmartDashboard.getEntry(pathRoot + "/time").getNumber(0).doubleValue();

    Rotation2d startHeading =
        Rotation2d.fromRadians(
            SmartDashboard.getEntry(pathRoot + "/start_heading").getNumber(0).doubleValue());

    Rotation2d endHeading =
        Rotation2d.fromRadians(
            SmartDashboard.getEntry(pathRoot + "/end_heading").getNumber(0).doubleValue());

    boolean stopAtEnd = SmartDashboard.getEntry(pathRoot + "/stop_at_end").getBoolean(true);

    double maxAcceleration = SmartDashboard.getEntry(pathRoot + "/max_acceleration").getNumber(5.0).doubleValue();

    Translation2d[] control_points = new Translation2d[4];

    for (int i = 0; i < 4; i++) {
      String controlPointRoot = pathRoot + "/control_point" + i;

      double x = SmartDashboard.getEntry(controlPointRoot + "/X").getNumber(0).doubleValue();
      double y = SmartDashboard.getEntry(controlPointRoot + "/Y").getNumber(0).doubleValue();

      control_points[i] = new Translation2d(x, y);
    }

    BezierCurve curve = new BezierCurve(control_points);

    return new Path(curve, startHeading, endHeading, 0.0, 0.0, 5.0, pathIndex, timeSeconds);
  }
}
