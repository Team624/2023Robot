// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

/** Represents a cubic bezier curve. */
public class BezierCurve {
  Translation2d[] p;
  double distance;

  public BezierCurve(Translation2d[] controlPoints) {
    this.p = controlPoints;
    this.distance = calculateDistance();
  }

  public Translation2d interpolate(double t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    // Math from https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves
    double x =
        Math.pow((1 - t), 3) * p[0].getX()
            + 3 * Math.pow((1 - t), 2) * t * p[1].getX()
            + 3 * (1 - t) * Math.pow(t, 2) * p[2].getX()
            + Math.pow(t, 3) * p[3].getX();

    double y =
        Math.pow((1 - t), 3) * p[0].getY()
            + 3 * Math.pow((1 - t), 2) * t * p[1].getY()
            + 3 * (1 - t) * Math.pow(t, 2) * p[2].getY()
            + Math.pow(t, 3) * p[3].getY();

    return new Translation2d(x, y);
  }

  // Approximate distance by splitting path into several segments
  private double calculateDistance() {
    final int NUMBER_OF_SEGMENTS = 50;

    Translation2d lastPoint = p[0];
    double distance = 0;

    for (int i = 1; i <= NUMBER_OF_SEGMENTS; i++) {
      double t = (double) i / NUMBER_OF_SEGMENTS;

      Translation2d nextPoint = interpolate(t);

      distance += lastPoint.getDistance(nextPoint);

      lastPoint = nextPoint;
    }

    return distance;
  }

  public double getDistance() {
    return distance;
  }
}
