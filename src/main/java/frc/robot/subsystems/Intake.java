// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor;
  // private PWMSparkMax intakeMotor;

  private GenericEntry currentIntakeEntry;
  private ShuffleboardTab intakeTab;
  // private CANSparkMax intakeMotorCAN;

  public Intake() {

    intakeMotor = new CANSparkMax(Constants.Intake.intakeMotor, MotorType.kBrushless);

    intakeMotor.setInverted(true);

    intakeMotor.setSmartCurrentLimit(10);

    intakeTab = Shuffleboard.getTab("Intake");

    currentIntakeEntry =
        intakeTab.add("Current Output Intake", intakeMotor.getOutputCurrent()).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentIntakeEntry.setDouble(getCurrentIntake());
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public double getCurrentIntake() {
    return intakeMotor.getOutputCurrent();
  }
}
