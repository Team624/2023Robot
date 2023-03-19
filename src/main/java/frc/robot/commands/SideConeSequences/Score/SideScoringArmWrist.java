// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SideCone.Score;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Wrist.SetWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideScoringArmWrist extends ParallelCommandGroup {
  /** Creates a new ArmWrist. */
  private final Arm m_Arm;

  private final Wrist m_Wrist;

  public SideScoringArmWrist(Arm arm, Wrist wrist, int i, boolean right) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_Arm = arm;
    this.m_Wrist = wrist;

    Rotation2d[] armPos = {Constants.Arm.ARM_SETPOINT_MID, Constants.Arm.ARM_SETPOINT_HIGH};
    Rotation2d newWristPos;
    Rotation2d newArmpos;

    if (right) {
      newWristPos = Constants.Wrist.wrist_zero;
    } else {
      newWristPos = Constants.Wrist.wrist_upright_cone_intake;
    }
    if (i == 1) {
      newArmpos = Constants.Arm.ARM_SETPOINT_PREHIGH_SCORE;
    } else {
      newArmpos = armPos[i];
    }

    addCommands(new SetArm(arm, newArmpos), new SetWrist(wrist, newWristPos));
  }
}
