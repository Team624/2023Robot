// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UprightCone.Score;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetpointUprightScore extends SequentialCommandGroup {

  /** Creates a new ArmTelescopeWrist. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;

  // private final ledControl m_led;

  public SetpointUprightScore(
      /** ledControl led, */
      Arm arm, Telescope telescope, Wrist wrist, int i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;

    Rotation2d[] armPos = {Constants.Arm.ARM_SETPOINT_MID, Constants.Arm.ARM_SETPOINT_HIGH};

    double[] telePos = {
      Constants.Telescope.TELESCOPE_SETPOINT_MID, Constants.Telescope.TELESCOPE_SETPOINT_HIGH
    };
    Rotation2d[] wristPos = {
      Constants.Wrist.wrist_zero,
      Constants.Wrist.wrist_upright_cone_Score,
      Constants.Wrist.wrist_cone_intake
    };

    if (m_Arm.getAbsoluteRotation().getDegrees() < 180) {

      Command command = new SetTelescope(telescope, 0.0);
      command.schedule();
    }

    addCommands(new UprightScoreArmWrist(arm, wrist, i), new SetTelescope(telescope, telePos[i]));
  }
}
