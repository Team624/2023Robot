// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class VisionApirlTags extends CommandBase {
  /** Creates a new VisionApirlTags. */

  private final Drivetrain m_drivetrain;
  private final Limelight m_limelight;
  private final DoubleSupplier m_translationXSupplier;

  private SlewRateLimiter filterX = new SlewRateLimiter(4.5);




  public VisionApirlTags(Drivetrain drivetrain,Limelight limelight, DoubleSupplier translationXSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_translationXSupplier = translationXSupplier;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.skewApril_pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thVelocity = 0;
    double yVelocity = 0;

    double horiz_distance = m_limelight.get_displacement()[0];
    double skew_angle = m_limelight.getSkew();

    if (horiz_distance<0){
      yVelocity = -1.5;
    }
    else{
      yVelocity=1.5;
    }
    thVelocity = getSkewPID(skew_angle);

    double xVelocity = m_translationXSupplier.getAsDouble();

    xVelocity = filterX.calculate(xVelocity);

    m_drivetrain.drive(new Translation2d(xVelocity,yVelocity), thVelocity, true, true);


    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getSkewPID(double wantedDeltaAngle) {
    return m_drivetrain.skewApril_pid.calculate(m_drivetrain.getYaw().getDegrees(),
        m_drivetrain.getYaw().getDegrees() + wantedDeltaAngle);
  }

  
}
