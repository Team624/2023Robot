// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Wrist.SetWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmTelescopeWrist extends SequentialCommandGroup {
  /** Creates a new ArmTelescopeWrist. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;

  public ArmTelescopeWrist(Arm arm, Telescope telescope, Wrist wrist, int i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;

    // bot, low, mid, top
    double[] wristPos = {0,1,2,3};
    double[] armPos = {0,1,2,3};
    double[] telePos = {0,1,2,3};
    addCommands(new SetArm(arm, armPos[i]), new SetTelescope(telescope, telePos[i]), new SetWrist(wrist, wristPos[i]));
  }
}
