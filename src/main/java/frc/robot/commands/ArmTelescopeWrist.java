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
public class ArmTelescopeWrist extends SequentialCommandGroup {
  public enum Setpoint {
    FUNNEL,
    CONE_INTAKE,
    CUBE_INTAKE,
    MID,
    HIGH
  }

  /** Creates a new ArmTelescopeWrist. */
  private final Arm m_Arm;

  private final Telescope m_Telescope;
  private final Wrist m_Wrist;
  // private final ledControl m_led;

  public ArmTelescopeWrist(
      /** ledControl led, */
      Arm arm, Telescope telescope, Wrist wrist, int i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.m_Arm = arm;
    this.m_Telescope = telescope;
    this.m_Wrist = wrist;
    // this.m_led = led;

    // retract = 0
    // IntakeCONE = 1
    // IntakeCUBE = 2
    // mid = 3
    // High = 4

    // if(!m_led.cone){
    //   if(i==1){
    //     i=2;
    //   }
    // }

    Rotation2d[] armPos = {
      Constants.Arm.ARM_SETPOINT_FUNNEL,
      Constants.Arm.ARM_SETPOINT_CONE_INTAKE,
      Constants.Arm.ARM_SETPOINT_CUBE_INTAKE,
      Constants.Arm.ARM_SETPOINT_MID,
      Constants.Arm.ARM_SETPOINT_HIGH
    };

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

    if (m_Arm.recentFunnel) {
      addCommands(
          new SetTelescope(telescope, 0.5),
          new SetWristCommand(wrist, 0.8),
          new SetArm(arm, armPos[i]),
          new SetWristCommand(wrist, wristPos[i]),
          new SetTelescope(telescope, telePos[i]));
    } else {
      addCommands(new SetArm(arm, armPos[i]), new TelescopeWrist(telescope, wrist, i));
    }

    m_Arm.recentFunnel = false;
  }
}
