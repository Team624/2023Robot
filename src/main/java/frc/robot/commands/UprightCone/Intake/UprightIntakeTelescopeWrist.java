// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UprightCone.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Wrist.SetWrist;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UprightIntakeTelescopeWrist extends ParallelCommandGroup {
  /** Creates a new TelescopeWrist. */
  private final Telescope m_Telescope;

  private final Wrist m_Wrist;

  public UprightIntakeTelescopeWrist(Telescope telescope, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Telescope = telescope;
    this.m_Wrist = wrist;
    addCommands(
        new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_UPRIGHT_CONE_INTAKE),
        new SetWrist(wrist, Constants.Wrist.wrist_upright_cone_Score));
  }
}
