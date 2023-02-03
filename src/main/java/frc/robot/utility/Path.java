// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Contains data and functions for an autonomous path. */
public class Path {
  private BezierCurve curve;
  private Rotation2d startHeading;
  private Rotation2d endHeading;
  private double timeSeconds;
  private int pathId;

  public Path(BezierCurve curve, Rotation2d startHeading, Rotation2d endHeading, int pathId, double timeSeconds) {
    this.curve = curve;
    this.pathId = pathId;
    this.timeSeconds = timeSeconds;

    this.startHeading = startHeading;
    this.endHeading = endHeading;
  }

  public int getPathId() {
    return pathId;
  }

  public double getSeconds() {
    return timeSeconds;
  }

  public Pose2d interpolate(double seconds) {
    double t = seconds / this.timeSeconds;

    if (t < 0) t = 0;
    if (t > 1) t = 1;

    Translation2d translation = curve.interpolate(t);

    double errorA = endHeading.getRadians() - startHeading.getRadians();
    double errorB = errorA - (Math.PI * 2);
    double errorC = errorA + (Math.PI * 2);
        
    double heading_diff = Math.abs(errorB) < Math.abs(errorC) ? errorB : errorC;
    heading_diff = Math.abs(errorA) < Math.abs(heading_diff) ? errorA : heading_diff;

    double rotation = MathUtil.angleModulus(startHeading.getRadians() + heading_diff * t);

    return new Pose2d(translation, Rotation2d.fromRadians(rotation));
  }

  public double getVelocity(double seconds) {
    // TODO: Implement max acceleration for smooth motion
    return curve.getDistance() / this.timeSeconds;
  }

  public BezierCurve getBezierCurve() {
    return curve;
  }
}
