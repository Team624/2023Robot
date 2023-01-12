// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.auton.AutonPathCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  private Auton auton;

  private SequentialCommandGroup commandGroup;
  private final Drivetrain m_drivetrainSubsystem;

  public AutonomousDrive(Drivetrain drivetrainSubsystem, Auton auton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.auton = auton;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false " + "-1");
    m_drivetrainSubsystem.setAuton(true);
    m_drivetrainSubsystem.stopAuton = false;
    m_drivetrainSubsystem.setPose();
    commandGroup = new SequentialCommandGroup();
    for (int i = 0; i < auton.getPathCount(); i++) {
      commandGroup.addCommands(new AutonPathCommand(m_drivetrainSubsystem, auton.auton[i], auton));
    }
    // this.alongWith(commandGroup);

    // commandGroup.schedule(false);
    commandGroup.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
