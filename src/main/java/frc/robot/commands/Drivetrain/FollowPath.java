// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Path;
import frc.robot.utility.PathPoint;

public class FollowPath extends CommandBase {
  private final Drivetrain drivetrain;

  private Path path;
  private int currentPointIndex = 0;
  private PathPoint point;

  public FollowPath(Drivetrain drive, Path path) {
    this.drivetrain = drive;
    this.path = path;
    this.point = path.getPoint(currentPointIndex);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting path " + path.getPathId());
    updateNTPoint();
    updateNTFinishedPath(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isWithinTolerance() && !nextPoint() || currentPointIndex >= path.getLength() - 1) return;

    double[] errors = getErrors();

    double vx = point.getVx() + errors[0];
    double vy = point.getVy() + errors[1];
    double vHeading = applyAutonRotationPID(point.getHeading());

    drivetrain.drive(new Translation2d(vx, vy), vHeading, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    updateNTFinishedPath(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("Distance: " + getDistance());
    // if (getDistance() > 1.0) {
    //   // Error is too high. Emergency stop path.
    //   System.err.println(
    //       "EMERGENCY STOP PATH " + path.getPathId() + " AT POINT " + currentPointIndex);
    //   drivetrain.stop();
    //   return true;
    // }

    return (currentPointIndex >= path.getLength() - 1);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }

  // Increments the current point if one exists.
  // Returns whether there was another point.
  private boolean nextPoint() {
    if (currentPointIndex + 1 >= path.getLength()) return false;

    currentPointIndex++;

    updateNTPoint();

    point = path.getPoint(currentPointIndex);

    System.out.println("Starting point " + currentPointIndex);

    return true;
  }

  // Updates NetworkTables with the current point.
  private void updateNTPoint() {
    SmartDashboard.getEntry("/pathTable/status/point").setNumber(currentPointIndex);
  }

  // Updates NetworkTables with the completion status of the path.
  private void updateNTFinishedPath(boolean finished) {
    SmartDashboard.getEntry("/pathTable/status/finishedPath")
        .setString(finished + " " + path.getPathId());
  }

  // Checks if the distance from the current point is within the tolerance
  private boolean isWithinTolerance() {
    return getDistance() <= point.getTolerance();
  }

  // Calculates the distance from the current point.
  private double getDistance() {
    double[] currentPos = drivetrain.getSwervePose();
    double pointX = point.getX();
    double pointY = point.getY();

    double diffX = Math.abs(pointX - currentPos[0]);
    double diffY = Math.abs(pointY - currentPos[1]);

    double distance = Math.hypot(diffX, diffY);

    return distance;
  }

  // Calculates the PID for durning to the given heading
  private double applyAutonRotationPID(double heading) {
    double wantedAngle = drivetrain.normalizeNuclearBombs(heading);

    double errorA =
      drivetrain.normalizeNuclearBombs(wantedAngle - drivetrain.normalizeNuclearBombs(drivetrain.getYaw().getRadians()));
    double errorB = errorA - (Math.PI * 2);
    double errorC = errorA + (Math.PI * 2);
    
    double wantedDeltaAngle = Math.abs(errorB) < Math.abs(errorC) ? errorB : errorC;
    wantedDeltaAngle = Math.abs(wantedDeltaAngle) < Math.abs(errorA) ? wantedDeltaAngle : errorA;

    System.out.println("Wanted: " + wantedAngle);
    System.out.println("Current: " + drivetrain.normalizeNuclearBombs(drivetrain.getYaw().getRadians()));
    System.out.println("Error: " + wantedDeltaAngle);
    System.out.println("Selected " + (wantedAngle == errorA ? "A" : wantedAngle == errorB ? "B" : "C"));

    double thVelocity =
        drivetrain.autonPoint_pidPathRotation.calculate(
            drivetrain.getYaw().getRadians(),
            drivetrain.getYaw().getRadians() + wantedDeltaAngle);

    return thVelocity;
  }

  // Returns the x and y distance from the nearest point on the line.
  private double[] getErrors() {
    Pose2d currentPose = drivetrain.getPose();
    PathPoint nextPoint = path.getPoint(currentPointIndex + 1);

    Translation2d nearestPoint =
        getClosestPointOnLine(
            point.getX(),
            point.getY(),
            nextPoint.getX(),
            nextPoint.getY(),
            currentPose.getX(),
            currentPose.getY());

    double[] result = {
      nearestPoint.getX() - currentPose.getX(), nearestPoint.getY() - currentPose.getY()
    };
    return result;
  }

  // Returns the neareset point on a line defined by 2 point.
  private Translation2d getClosestPointOnLine(
      double point1X,
      double point1Y,
      double point2X,
      double point2Y,
      double point3X,
      double point3Y) {
    double perpSlope = -(point2X - point1X) / (point2Y - point1Y);

    // Catch the case of the perpSlope being undefined
    if (Double.isInfinite(perpSlope)) {
      return new Translation2d(point3X, point1Y);
    }

    double point4X = point3X + 1;
    double point4Y = point3Y + perpSlope;

    double[] L1 = line(point1X, point1Y, point2X, point2Y);
    double[] L2 = line(point3X, point3Y, point4X, point4Y);

    double[] result = intersection(L1, L2);
    return new Translation2d(result[0], result[1]);
  }

  // Extra stuff only used in getClosestPointOnLine()
  private double[] intersection(double[] L1, double[] L2) {
    double D = L1[0] * L2[1] - L1[1] * L2[0];
    double Dx = L1[2] * L2[1] - L1[1] * L2[2];
    double Dy = L1[0] * L2[2] - L1[2] * L2[0];

    // Might return infinity if there is no intersection!
    double x = Dx / D;
    double y = Dy / D;
    double[] result = {x, y};
    return result;
  }

  // Extra stuff only used in getClosestPointOnLine()
  private double[] line(double point1X, double point1Y, double point2X, double point2Y) {
    double A = point1Y - point2Y;
    double B = point2X - point1X;
    double C = (point1X * point2Y) - (point2X * point1Y);
    double[] result = {A, B, -C};
    return (result);
  }
}
