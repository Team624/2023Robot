package frc.robot.commands.Drivetrain;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ReflectiveAlign extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_translationXSupplier;
  private SlewRateLimiter filterX = new SlewRateLimiter(4.5);

  private Limelight limelight;
  private ArrayList<Double> values;
  private boolean manual;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;

  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private final PIDController yController =
      new PIDController(0.15, 0.0, 0.0);

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
  new TrapezoidProfile.Constraints(2 * Math.PI, Math.pow(2 * Math.PI, 2));

  private final ProfiledPIDController omegaController =
    new ProfiledPIDController(Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KP, 0, 0, OMEGA_CONSTRAINTS);

  public ReflectiveAlign(Drivetrain m_drivetrain, Limelight limelight,DoubleSupplier translationXSupplier, boolean manual) {
    this.limelight = limelight;
    this.m_drivetrain = m_drivetrain;
    this.m_translationXSupplier=translationXSupplier;
    values =  new ArrayList<Double>();
    yController.setSetpoint(0);
    yController.setTolerance(.01);
    omegaController.setTolerance(Units.degreesToRadians(2));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    this.manual = manual;

    this.addRequirements(m_drivetrain, limelight);
  }

  @Override
  public void initialize() {
    Pose2d pose = m_drivetrain.getPose();
    omegaController.reset(pose.getRotation().getRadians());
    System.out.println("Running reflective align");

    limelight.setLEDs(true);
  }

  public void execute() {
    // if(limelight.getData().pipeline != 1) return;
    double currentAngle = limelight.getData().tx;

    System.out.println("Current angle: " + currentAngle);

    values.add(currentAngle);
    
    if(values.size() > 5){
      values.remove(0);
    }

    System.out.println("Average angle: " + getAverageAngle());

    double yFeedback = -yController.calculate(getAverageAngle());
    yFeedback = MathUtil.clamp(yFeedback, -0.5, 0.5);

    omegaController.setGoal(0.0);
    double omegaFeedback = omegaController.calculate(m_drivetrain.getPose().getRotation().getRadians());

    double vx = m_translationXSupplier.getAsDouble();
    vx = filterX.calculate(vx);

    m_drivetrain.drive(new ChassisSpeeds(vx, yFeedback, omegaFeedback), true, false);
  }

  public double getAverageAngle(){
    double sumAngles = 0;
    for(double angle: values){
      sumAngles+=angle;
    }
    return sumAngles/values.size();
  }

  @Override
  public void end(boolean interrupted){
    values.clear();
    limelight.setLEDs(false);
  }

  public boolean isFinished() {
    // return yController.atGoal();
    return limelight.getData().tv && yController.atSetpoint() && omegaController.atSetpoint() && !manual;
  }
}
