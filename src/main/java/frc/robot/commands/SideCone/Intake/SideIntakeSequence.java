// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SideCone.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
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

    // CUBE INTAKE == laying down
    // CONE INTAKE == upright

    // Double Substation = 0
    // retract = 1
    // IntakeCONE = 2
    // IntakeCUBE = 3
    // mid = 4
    // High = 5

    addCommands(
        new SetArm(arm, Constants.Arm.ARM_SETPOINT_PREINTAKE),
        new SideIntakeTelescopeWrist(telescope, wrist),
        new SetArm(arm, Constants.Arm.ARM_SETPOINT_SIDE_CONE_INTAKE));
  }
}
