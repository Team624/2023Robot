// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Auton {

  public Path[] auton;

  private Drivetrain drivetrain;
  public boolean isAuton = false;

  private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

  private GenericEntry autoChoiceGet =
      autoTab
          .add("Auton Choice", 0)
          .withPosition(0, 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();

  // States so that we don't schedule more than once

  public Auton(Drivetrain drivetrain) {
    auton = getAuto();
    this.drivetrain = drivetrain;
  }

  public void updatePaths() {
    auton = getAuto();
  }

  public void sendAutoChoice() {
    Number autoChoice = autoChoiceGet.getDouble(0.0);
    SmartDashboard.putNumber("/auto/select", (double) autoChoice);
  }

  public Path[] getAuto() {
    int pathCount = getPathCount();
    Path[] auto = new Path[pathCount];
    for (int i = 0; i < pathCount; i++) {
      auto[i] = cyclePath(i);
    }
    return auto;
  }

  private Path cyclePath(int pathNum) {
    PathPoint[] points = cyclePoints(pathNum);
    return new Path(points, pathNum);
  }

  private PathPoint[] cyclePoints(int pathNum) {
    int pathLength = getPathLength(pathNum);
    PathPoint[] points = new PathPoint[pathLength];
    for (int i = 0; i < pathLength; i++) {
      points[i] = setPoint(pathNum, i);
    }
    return points;
  }

  private PathPoint setPoint(int pathNum, int pointNum) {
    String pathString = "/pathTable/path" + pathNum + "/point" + pointNum + "/";
    PathPoint point =
        new PathPoint(
            SmartDashboard.getEntry(pathString + "X").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Y").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Vx").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Vy").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Heading").getDouble(0.0),
            SmartDashboard.getEntry(pathString + "Tolerance").getDouble(0.0));
    return point;
  }

  public int getPathCount() {
    return SmartDashboard.getEntry("/pathTable/numPaths").getNumber(0).intValue();
  }

  private int getPathLength(int pathNum) {
    return SmartDashboard.getEntry("/pathTable/path" + pathNum + "/numPoints")
        .getNumber(0)
        .intValue();
  }

  public void setState(boolean state) {
    isAuton = state;
    SmartDashboard.putBoolean("/auto/state", state);
  }

  public int getStartPathIndex() {
    return SmartDashboard.getEntry("/pathTable/startPathIndex").getNumber(-1).intValue();
  }
}
