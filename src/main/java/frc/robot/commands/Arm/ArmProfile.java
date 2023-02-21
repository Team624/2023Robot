// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmProfile extends TrapezoidProfileCommand {
  /** Creates a new ArmProfile. */
  public ArmProfile(Arm m_Arm, double setPoint) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 1.1),
            // Goal state
            new TrapezoidProfile.State(setPoint,0),
            // Initial state
            new TrapezoidProfile.State(m_Arm.getBoreEncoder(),0)),
        state -> {
          // Use current trajectory state here
          m_Arm.ArmProfile(state);
        },
        m_Arm);
  }
}
