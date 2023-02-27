// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // private CANSparkMax intakeMotor;
  private PWMSparkMax intakeMotor;

  private RelativeEncoder intakeEncoder;

  private SparkMaxPIDController intakePidController;

  private double IntakeP;
  private double IntakeI;
  private double IntakeD;

  public Intake() {

    // intakeMotor = new CANSparkMax(Constants.Intake.intakeMotor, MotorType.kBrushless);

    intakeMotor = new PWMSparkMax(0);

    intakeMotor.setInverted(true);

    // intakePidController = intakeMotor.getPIDController();
    // intakeMotor.setIdleMode(IdleMode.kBrake);
    // IntakeP = frc.robot.Constants.Intake.P;
    // IntakeI = frc.robot.Constants.Intake.I;
    // IntakeD = frc.robot.Constants.Intake.D;

    // intakePidController.setP(IntakeP);
    // intakePidController.setI(IntakeI);
    // intakePidController.setD(IntakeD);

    // intakePidController.setOutputRange(-1, 1);

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
}
