// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignwithTag extends SequentialCommandGroup {
  /** Creates a new AlignwithTag. */
  private final Drivetrain m_drivetrain;

  private final Limelight m_limelight;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_translationThSupplier;

  public AlignwithTag(
      Drivetrain drivetrain,
      Limelight limelight,
      DoubleSupplier xSup,
      DoubleSupplier ySup,
      DoubleSupplier thSup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_translationXSupplier = xSup;
    this.m_translationYSupplier = ySup;
    this.m_translationThSupplier = thSup;

    addCommands(
        new VisionAprilTags(
            m_drivetrain, m_limelight, m_translationXSupplier, m_translationThSupplier),
        new AprilTagTheta(
            m_drivetrain, m_limelight, m_translationXSupplier, m_translationYSupplier));
  }
}
