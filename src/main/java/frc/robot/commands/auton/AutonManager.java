// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.FollowPath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.BezierCurve;
import frc.robot.utility.Path;

// Gets states from NetworkTables (published by ros) during auton
public class AutonManager extends CommandBase {
  private Drivetrain drivetrain;
  private Path[] paths;
  private FollowPath currentFollowPathCommand;
  private int previousPath = -1;

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
    if (!startNTPath()) {
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setAuton(false);

    SmartDashboard.putBoolean("/auto/state", false);

    System.out.println("Auton ended!");
  }

  // Starts the path specified by ROS in NetworkTables
  // Returns whether a path was started
  private boolean startNTPath() {
    if (currentFollowPathCommand != null && currentFollowPathCommand.isScheduled()) return false;

    int index = SmartDashboard.getEntry("/pathTable/startPathIndex").getNumber(-1).intValue();

    if (index < 0 || index >= paths.length || index <= previousPath) return false;

    FollowPath followPathCommand = new FollowPath(this.drivetrain, this.paths[index]);

    currentFollowPathCommand = followPathCommand;

    previousPath = index;

    // Use deadlineWith to stop when AutonManager stops.
    followPathCommand.schedule();

    SmartDashboard.getEntry("/pathTable/startPathIndex").setNumber(-1);

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

    double timeSeconds = 
      SmartDashboard.getEntry(pathRoot + "/time")
        .getNumber(0)
        .doubleValue();

    Rotation2d startHeading =
      Rotation2d.fromRadians(
        SmartDashboard.getEntry(pathRoot + "/start_heading")
          .getNumber(0)
          .doubleValue());

    Rotation2d endHeading =
      Rotation2d.fromRadians(
        SmartDashboard.getEntry(pathRoot + "/end_heading")
          .getNumber(0)
          .doubleValue());

    Translation2d[] control_points = new Translation2d[4];

    for (int i = 0; i < 4; i++) {
      String controlPointRoot = pathRoot + "/control_point" + i;

      double x = SmartDashboard.getEntry(controlPointRoot + "/X")
        .getNumber(0)
        .doubleValue();
      double y = SmartDashboard.getEntry(controlPointRoot + "/Y")
        .getNumber(0)
        .doubleValue();

      control_points[i] = new Translation2d(x, y);
    }

    BezierCurve curve = new BezierCurve(control_points);
        
    return new Path(curve, startHeading, endHeading, pathIndex, timeSeconds);
  }
}
