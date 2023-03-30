package frc.robot.commands.Drivetrain;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class ReflectiveAlign extends CommandBase {

  private final Drivetrain m_drivetrain;

  private Limelight limelight;
  private ArrayList<Double> values;

  public static final double MaxVel = Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;

  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MaxVel, 2);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(0.1, 0.0, 0.0, Y_CONSTRAINTS);

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
  new TrapezoidProfile.Constraints(2 * Math.PI, Math.pow(2 * Math.PI, 2));

  private final ProfiledPIDController omegaController =
    new ProfiledPIDController(Constants.Limelight.kRotationP, 0, 0, OMEGA_CONSTRAINTS);

  public ReflectiveAlign(Drivetrain m_drivetrain, Limelight limelight) {
    this.limelight = limelight;
    this.m_drivetrain = m_drivetrain;
    values =  new ArrayList<Double>();
    yController.setGoal(0);
    yController.setTolerance(.01);

    omegaController.setGoal(0);

    this.addRequirements(m_drivetrain, limelight);
  }

  @Override
  public void initialize() {
    yController.reset(limelight.getData().tx);
    omegaController.reset(m_drivetrain.getPose().getRotation().getRadians());
    System.out.println("Running reflective align");
  }

  public void execute() {
    if(limelight.getData().pipeline != 1) return;
    double currentAngle = limelight.getData().tx;

    values.add(currentAngle);
    
    if(values.size() > 5){
      values.remove(0);
    }

    double yFeedback = -yController.calculate(getAverageAngle());
    yFeedback = MathUtil.clamp(yFeedback, -0.5, 0.5);

    double omegaFeedback = omegaController.calculate(m_drivetrain.getPose().getRotation().getRadians());

    m_drivetrain.drive(new ChassisSpeeds(0, yFeedback, omegaFeedback), true, false);
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
    // limelight.changePipelined(0);
    // UpdatePose.keepRunning = true;
  }

  public boolean isFinished() {
    return yController.atGoal();
  }
}
