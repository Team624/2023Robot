// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SideConeSequences.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideIntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;

  public SideIntakeSequence(Arm arm, Telescope telescope, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;

    if (m_Arm.getAbsoluteRotation().getDegrees() > 180) {
      // Command command = new SetTelescope(telescope, 0.0);
      // command.schedule();

      new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_ZERO);
    }

    addCommands(
        new SetArm(arm, Constants.Arm.ARM_SETPOINT_PREINTAKE),
        new SideIntakeArmWrist(arm, wrist),
        new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_SIDE_CONE_INTAKE));
  }
}
