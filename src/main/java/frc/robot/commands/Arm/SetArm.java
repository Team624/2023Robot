// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  private final Arm m_Arm;

  private final double m_setPoint;



  private static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1.0, 3);

  private final ProfiledPIDController armController =
      new ProfiledPIDController(3, 0, 0, ARM_CONSTRAINTS);

  public SetArm(Arm arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    this.m_setPoint = setpoint;

    armController.setTolerance(0.5);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armController.reset(m_Arm.getBoreEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d setPoint2d = new Rotation2d(m_setPoint);
    armController.setGoal(m_setPoint);
    double armVel = armController.calculate(m_Arm.getBoreEncoder());

    m_Arm.setArmCommand(armVel,setPoint2d);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armController.atGoal()){
      return true;
    }
    return false;
  }
}
