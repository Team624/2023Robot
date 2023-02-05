// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor;

  private Solenoid intakeSolenoid;

  private RelativeEncoder intakeEncoder;
  private SparkMaxPIDController intakePID;

  private double P;
  private double I;
  private double D;

  public Intake() {
    intakeMotor = new CANSparkMax(frc.robot.Constants.Intake.intakeMotor, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeEncoder = intakeMotor.getEncoder();
    // intakeMotor.setIdleMode(IdleMode.kBrake);
    intakePID = intakeMotor.getPIDController();

    P = frc.robot.Constants.Intake.P;
    I = frc.robot.Constants.Intake.I;
    D = frc.robot.Constants.Intake.D;

    intakePID.setP(P);
    intakePID.setI(I);
    intakePID.setD(D);

    intakePID.setOutputRange(-1, 1);

    intakeSolenoid =
        new Solenoid(30, PneumaticsModuleType.CTREPCM, frc.robot.Constants.Intake.intakeSolenoidID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void deploySolenoids() {
    intakeSolenoid.set(true);
  }

  public void retractSolenoids() {
    intakeSolenoid.set(false);
  }
}
