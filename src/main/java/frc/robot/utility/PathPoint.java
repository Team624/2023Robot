package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathPoint {
  private double x, y, vx, vy, heading, tolerance;

  public PathPoint(double x, double y, double vx, double vy, double heading, double tolerance) {
    this.x = x;
    this.y = y;
    this.vx = vx;
    this.vy = vy;
    this.heading = heading;
    this.tolerance = tolerance;
  }

  public void setX(double x) {
    this.x = x;
  }

  public void setY(double y) {
    this.y = y;
  }

  public void setVx(double vx) {
    this.vx = vx;
  }

  public void setVy(double vy) {
    this.vy = vy;
  }

  public void setHeading(double h) {
    this.heading = h;
  }

  public void setTolerance(double v) {
    this.tolerance = v;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getVx() {
    return vx;
  }

  public double getVy() {
    return vy;
  }

  public double getHeading() {
    return heading;
  }

  public double getTolerance() {
    return tolerance;
  }

  public Pose2d getPose() {
    return new Pose2d(this.x, this.y, new Rotation2d(heading));
  }
}
