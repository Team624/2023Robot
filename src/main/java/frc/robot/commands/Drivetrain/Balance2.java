// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Balance2 extends CommandBase {
  /** Creates a new Balance2. */
  private final Drivetrain m_Drivetrain;

  private double balanaceEffort;
  private double turningEffort;
  private boolean onGround;
  private boolean reversed;

  public Balance2(Drivetrain drivetrain, boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain = drivetrain;
    this.reversed = reversed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onGround = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDegrees = m_Drivetrain.getPitch();
    if (Math.abs(angleDegrees) >= Constants.Autonomous.AUTO_BALANCE_GROUND_ANGLE_THRESHOLD) {
      onGround = false;
    }
    if (onGround) {
      m_Drivetrain.drive(
          new ChassisSpeeds(
              Constants.Autonomous.AUTO_BALANCE_GROUND_SPEED * (reversed ? -1 : 1), 0, 0),
          true,
          false);

    }
    else{
      turningEffort =
      m_Drivetrain.calculateThetaSupplier(() -> Constants.Autonomous.angleSetPoint).getAsDouble();
  balanaceEffort =
      (Constants.Autonomous.balancedAngle - m_Drivetrain.getPitch()) * Constants.Autonomous.kP;
  m_Drivetrain.drive(new ChassisSpeeds(balanaceEffort, 0, turningEffort), false, true);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.stopWithX();
    setNTState(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_Drivetrain.getPitch());
    return !onGround && Math.abs(m_Drivetrain.getPitch()) < 2;
  }

  private void setNTState(boolean state) {
    SmartDashboard.getEntry("/auto/balance/state").setBoolean(state);
  }
}
