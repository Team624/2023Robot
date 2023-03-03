// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Contains data and functions for an autonomous path. */
public class Path {
  public class State {
    public Pose2d pose;
    public double velocity;
    public Rotation2d wantedHeading;

    public State(Pose2d pose, double velocity, Rotation2d wantedHeading) {
      this.pose = pose;
      this.velocity = velocity;
      this.wantedHeading = wantedHeading;
    }
  }

  private BezierCurve curve;
  private Rotation2d startHeading;
  private Rotation2d endHeading;
  private double timeSeconds;
  private int pathId;
  private double maxAcceleration;
  private double startVelocity;
  private boolean stopAtEnd;
  private double maxVelocity;

  private TrapezoidProfile profile;

  public Path(
      BezierCurve curve,
      Rotation2d startHeading,
      Rotation2d endHeading,
      double startVelocity,
      boolean stopAtEnd,
      double maxAcceleration,
      int pathId,
      double timeSeconds) {
    this.curve = curve;
    this.pathId = pathId;
    this.timeSeconds = timeSeconds;
    this.startHeading = startHeading;
    this.endHeading = endHeading;
    this.startVelocity = startVelocity;
    this.stopAtEnd = stopAtEnd;
    this.maxAcceleration = maxAcceleration;

    this.maxVelocity = calculateMaxVelocity();

    this.profile =
        new TrapezoidProfile(
            new Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(curve.getDistance(), getEndVelocity()),
            new TrapezoidProfile.State(0, startVelocity));

    System.out.println("Path number: " + pathId);
    System.out.println("startVelocity: " + startVelocity);
    System.out.println("stopAtEnd: " + stopAtEnd);
    System.out.println("maxVelocity: " + maxVelocity);
    System.out.println("maxAcceleration: " + maxAcceleration);
    System.out.println("endVelocity: " + getEndVelocity());
    System.out.println("Distance: " + curve.getDistance());
    System.out.println("time: " + timeSeconds);
  }

  public int getPathId() {
    return pathId;
  }

  public double getSeconds() {
    return timeSeconds;
  }

  public Pose2d interpolate(double t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    Translation2d translation = curve.interpolate(t);
    Rotation2d direction = curve.getFirstDerivative(t).getAngle();

    // direction = new Rotation2d(Math.PI - direction.getRadians());

    return new Pose2d(translation, direction);
  }

  // private double calculateMaxVelocity() {
  //   double v1 = Math.min(startVelocity, endVelocity);
  //   double v2 = Math.max(startVelocity, endVelocity);

  //   System.out.println(v1 + " " + v2);

  //   // Arnav code
  //   double dv = v2 - v1;
  //   double tIn = dv / maxAcceleration;
  //   double minx = (timeSeconds - .5 * tIn) * dv + v1 * timeSeconds;
  //   double xEx = curve.getDistance() - minx;
  //   double newT = timeSeconds - tIn;
  //   double xmax = maxAcceleration * newT * newT / 4;
  //   double xdiff = xmax - xEx;
  //   // get triangle for which xdiff is area

  //   System.out.println("xdiff: " + xdiff + " maxAccel: " + maxAcceleration);

  //   double tmid = 2 * Math.sqrt(xdiff / maxAcceleration);

  //   System.out.println("newT: " + newT + " tmid: " + tmid);

  //   double tplus = .5 * (newT - tmid);

  //   System.out.println(maxVelocity + " " + tIn + " " + tplus + " " + v1);

  //   return maxAcceleration * (tIn + tplus) + v1;
  // }

  public double calculateMaxVelocity() {
    if (!stopAtEnd) {
      double xEx = curve.getDistance() - startVelocity * timeSeconds;
      // double goofyArnavMaxAccel = xEx<0 ? -maxAcceleration : maxAcceleration;

      double goofyArnavMaxAccel = xEx < 0 ? -maxAcceleration : maxAcceleration;

      double areaT = timeSeconds * timeSeconds * .5 * goofyArnavMaxAccel - xEx;
      // v2=vo2+2ax

      double tnew = Math.sqrt(areaT * 2 / goofyArnavMaxAccel);
      return startVelocity + (goofyArnavMaxAccel * (timeSeconds - tnew));
    }

    double v1 = 0;
    double v2 = startVelocity;

    double dv = v2 - v1;
    double tIn = dv / maxAcceleration;
    double minx = (timeSeconds - .5 * tIn) * dv + v1 * timeSeconds;
    double xEx = curve.getDistance() - minx;
    double newT = timeSeconds - tIn;
    double xmax = maxAcceleration * newT * newT / 4;
    double xdiff = xmax - xEx;
    double tplus;
    // get triangle for which xdiff is area
    double tmid = 2 * Math.sqrt(xdiff / maxAcceleration);
    tplus = .5 * (newT - tmid);
    return maxAcceleration * (tIn + tplus) + v1;
  }

  public double getEndVelocity() {
    if (stopAtEnd) return 0;
    return maxVelocity;
  }

  public State getState(double seconds) {
    TrapezoidProfile.State profileState =
        profile.calculate(MathUtil.clamp(seconds, 0, this.timeSeconds));

    Pose2d pose = interpolate(seconds / this.timeSeconds);

    return new State(pose, profileState.velocity, this.endHeading);
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public boolean getStopAtEnd() {
    return stopAtEnd;
  }

  public BezierCurve getBezierCurve() {
    return curve;
  }
}
