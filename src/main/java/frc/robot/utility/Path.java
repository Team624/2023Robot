// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

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
  private double endVelocity;
  private double maxVelocity;

  private TrapezoidProfile profile;

  public Path(
      BezierCurve curve,
      Rotation2d startHeading,
      Rotation2d endHeading,
      double startVelocity,
      double endVelocity,
      double maxAcceleration,
      int pathId,
      double timeSeconds) {
    this.curve = curve;
    this.pathId = pathId;
    this.timeSeconds = timeSeconds;

    this.startHeading = startHeading;
    this.endHeading = endHeading;

    this.startVelocity = startVelocity;
    this.endVelocity = endVelocity;
    this.maxAcceleration = maxAcceleration;

    this.maxVelocity = calculateMaxVelocity();

    System.out.println(startVelocity + " " + endVelocity + " " + maxAcceleration);

    System.out.println("MAX VEL: " + maxVelocity);

    this.profile =
        new TrapezoidProfile(
            new Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(curve.getDistance(), endVelocity),
            new TrapezoidProfile.State(0, startVelocity));
  }

  public int getPathId() {
    return pathId;
  }

  public double getSeconds() {
    return timeSeconds;
  }

  public Pose2d interpolate(double t) {
    // double t = distance / curve.getDistance();

    if (t < 0) t = 0;
    if (t > 1) t = 1;

    Translation2d translation = curve.interpolate(t);

    // double errorA = endHeading.getRadians() - startHeading.getRadians();
    // double errorB = errorA - (Math.PI * 2);
    // double errorC = errorA + (Math.PI * 2);

    // double heading_diff = Math.abs(errorB) < Math.abs(errorC) ? errorB : errorC;
    // heading_diff = Math.abs(errorA) < Math.abs(heading_diff) ? errorA : heading_diff;

    return new Pose2d(translation, curve.getFirstDerivative(t).getAngle());
  }

  private double calculateMaxVelocity() {
    double v1 = Math.min(startVelocity, endVelocity);
    double v2 = Math.max(startVelocity, endVelocity);

    System.out.println(v1 + " " + v2);

    // Arnav code
    double dv = v2 - v1;
    double tIn = dv / maxAcceleration;
    double minx = (timeSeconds - .5 * tIn) * dv + v1 * timeSeconds;
    double xEx = curve.getDistance() - minx;
    double newT = timeSeconds - tIn;
    double xmax = maxAcceleration * newT * newT / 4;
    double xdiff = xmax - xEx;
    // get triangle for which xdiff is area
    
    System.out.println("xdiff: " + xdiff + " maxAccel: " + maxAcceleration);

    double tmid = 2 * Math.sqrt(xdiff / maxAcceleration);

    System.out.println("newT: " + newT + " tmid: " + tmid);

    double tplus = .5 * (newT - tmid);

    System.out.println(maxVelocity + " " + tIn + " " + tplus + " " + v1);

    return maxAcceleration * (tIn + tplus) + v1;
  }

  public State getState(double seconds) {
    TrapezoidProfile.State profileState = profile.calculate(MathUtil.clamp(seconds, 0, this.timeSeconds));

    System.out.println("Distance: " + profileState.position / curve.getDistance() + " Time: " + seconds / timeSeconds);

    Pose2d pose = interpolate(seconds / this.timeSeconds);

    return new State(pose, profileState.velocity, this.endHeading);
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public BezierCurve getBezierCurve() {
    return curve;
  }
}
