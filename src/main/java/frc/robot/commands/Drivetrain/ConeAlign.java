package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ConeAlign extends CommandBase {

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2);
  private final ProfiledPIDController xController =
      new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(8, 8);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);
  private final Drivetrain m_drivetrain;
  private Limelight vision;
  private int direction;

  public ConeAlign(Drivetrain drivetrain, Limelight visionAprilTags, boolean right) {
    if (right) {
      direction = 1;
    } else {
      direction = -1;
    }
    m_drivetrain = drivetrain;
    vision = visionAprilTags;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (vision.hasTarget()) {
      double[] distance_angle = vision.alignment_values();
      double[] current = m_drivetrain.getSwervePose();
      double apriltag2cone = 22 / 39.37; // inches to meters
      System.out.println(vision.getY());
      // Pose2d desiredPose =
      //     new Pose2d(
      //         m_drivetrain.getPose().getX(),
      //         vision.getY() + direction * apriltag2cone,
      //         new Rotation2d(0));
      xController.setGoal(m_drivetrain.getPose().getX());
      yController.setGoal(vision.getY() + direction * apriltag2cone);
      // omegaController.setGoal(desiredPose.getRotation().getRadians());
      System.out.println("In Cone Align");
      // double xSpeed = xController.calculate(current[0]);
      // if (xController.atGoal()) {
      //   xSpeed = 0;
      // }

      double ySpeed = yController.calculate(current[1]);
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      // double omegaSpeed = omegaController.calculate(current.getRotation().getRadians());
      // if (omegaController.atGoal()) {
      //   omegaSpeed = 0;
      // }
      System.out.println(yController.getPositionError());
      m_drivetrain.drive(new Translation2d(0, ySpeed), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return yController.atGoal();
  }
}
