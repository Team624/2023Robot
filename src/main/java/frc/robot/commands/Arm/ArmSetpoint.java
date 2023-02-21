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
public class ArmSetpoint extends TrapezoidProfileCommand {
  /** Creates a new ArmSetpoint. */
  public ArmSetpoint(Arm m_arm, double setpoint) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(1, 3),
            // End at desired position in meters; implicitly starts at 0
            new TrapezoidProfile.State(setpoint, 0)),
        // Pipe the profile state to the drive
        setpointState -> m_arm.ArmProfile(setpointState),
        // Require the drive
        m_arm);
    System.out.println(m_arm.getBoreEncoder());
  }
}
