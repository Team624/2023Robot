// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Wrist.SetWristCommand;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescopeWrist extends ParallelCommandGroup {
  /** Creates a new TopPosition. */
  private final Telescope m_Telescope;

  private final Wrist m_Wrist;

  public TelescopeWrist(Telescope telescope, Wrist wrist, int i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Telescope = telescope;
    this.m_Wrist = wrist;

    // Funnel = 0
    // IntakeCONE = 1
    // IntakeCUBE = 2
    // mid = 3
    // High = 4

    double[] telePos = {
      Constants.Telescope.TELESCOPE_SETPOINT_FUNNEL,
      Constants.Telescope.TELESCOPE_SETPOINT_CONE_INTAKE,
      Constants.Telescope.TELESCOPE_SETPOINT_CUBE_INTAKE,
      Constants.Telescope.TELESCOPE_SETPOINT_MID,
      Constants.Telescope.TELESCOPE_SETPOINT_HIGH
    };
    double[] wristPos = {
      Constants.Wrist.WRIST_SETPOINT_FUNNEL,
      Constants.Wrist.WRIST_SETPOINT_CONE_INTAKE,
      Constants.Wrist.WRIST_SETPOINT_CUBE_INTAKE,
      Constants.Wrist.WRIST_SETPOINT_MID,
      Constants.Wrist.WRIST_SETPOINT_HIGH
    };

    addCommands(new SetWristCommand(wrist, wristPos[i]), new SetTelescope(telescope, telePos[i]));
  }
}
