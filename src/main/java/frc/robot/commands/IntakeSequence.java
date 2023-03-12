// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Wrist.SetWristCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;

  public IntakeSequence(Arm arm, Telescope telescope, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;

    // CUBE INTAKE == laying down
    // CONE INTAKE == upright

    // Double Substation = 0
    // retract = 1
    // IntakeCONE = 2
    // IntakeCUBE = 3
    // mid = 4
    // High = 5

    Rotation2d[] armPos = {
      Constants.Arm.ARM_SETPOINT_DOUBLE_SUBSTATION,
      Constants.Arm.ARM_SETPOINT_FUNNEL,
      Constants.Arm.ARM_SETPOINT_CONE_INTAKE,
      Constants.Arm.ARM_SETPOINT_CUBE_INTAKE,
      Constants.Arm.ARM_SETPOINT_MID,
      Constants.Arm.ARM_SETPOINT_HIGH
    };

    double[] telePos = {
      Constants.Telescope.TELESCOPE_SETPOINT_DOUBLE_SUBSTATION,
      Constants.Telescope.TELESCOPE_SETPOINT_FUNNEL,
      Constants.Telescope.TELESCOPE_SETPOINT_CONE_INTAKE,
      Constants.Telescope.TELESCOPE_SETPOINT_CUBE_INTAKE,
      Constants.Telescope.TELESCOPE_SETPOINT_MID,
      Constants.Telescope.TELESCOPE_SETPOINT_HIGH
    };
    double[] wristPos = {
      Constants.Wrist.WRIST_SETPOINT_DOUBLE_SUBSTATION,
      Constants.Wrist.WRIST_SETPOINT_FUNNEL,
      Constants.Wrist.WRIST_SETPOINT_CONE_INTAKE,
      Constants.Wrist.WRIST_SETPOINT_MID,
      Constants.Wrist.WRIST_SETPOINT_HIGH
    };

    addCommands(
        new SetWristCommand(wrist, wristPos[3]),
        new SetTelescope(telescope, telePos[3]),
        new SetArm(arm, armPos[3]));
  }
}
