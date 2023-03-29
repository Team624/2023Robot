// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class Balance2 extends CommandBase {
  /** Creates a new Balance2. */
  private final Drivetrain m_Drivetrain;
  private double balanaceEffort; 
    private double turningEffort; 
  public Balance2(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain=drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turningEffort =
                m_Drivetrain
                        .calculateThetaSupplier(() -> Constants.Autonomous.angleSetPoint)
                        .getAsDouble();
        balanaceEffort =
                (Constants.Autonomous.balancedAngle - m_Drivetrain.getPitch())
                        * Constants.Autonomous.kP;
     m_Drivetrain.drive(new ChassisSpeeds(balanaceEffort, 0, turningEffort), false, true);
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
    return Math.abs(m_Drivetrain.getPitch()) < 2;
  }
  private void setNTState(boolean state) {
    SmartDashboard.getEntry("/auto/balance/state").setBoolean(state);
  }
}
