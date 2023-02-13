// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristMotor;

  private CANSparkMax intakeMotor;

  private RelativeEncoder intakEncoder;
  private RelativeEncoder wristEncoder;

  private SparkMaxPIDController intakePidController;
  private SparkMaxPIDController wristPidController;

  private double WristP;
  private double WristI;
  private double WristD;

  private double IntakeP;
  private double IntakeI;
  private double IntakeD;

  public Wrist() {

    wristMotor = new CANSparkMax(Constants.Wrist.WristMotor, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.WristIntake.WristIntakeMotor, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    wristEncoder = wristMotor.getEncoder();
    intakEncoder = intakeMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristPidController = wristMotor.getPIDController();
    intakePidController = intakeMotor.getPIDController();

    WristP = frc.robot.Constants.Wrist.P;
    WristI = frc.robot.Constants.Wrist.I;
    WristD = frc.robot.Constants.Wrist.D;

    IntakeP = frc.robot.Constants.WristIntake.P;
    IntakeI = frc.robot.Constants.WristIntake.I;
    IntakeD = frc.robot.Constants.WristIntake.D;

    wristPidController.setP(WristP);
    wristPidController.setI(WristI);
    wristPidController.setD(WristD);

    intakePidController.setP(IntakeP);
    intakePidController.setI(IntakeI);
    intakePidController.setD(IntakeD);

    wristPidController.setOutputRange(-1, 1);
    intakePidController.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void moveWrist(double speed) {
    wristMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void stopWrist() {
    wristMotor.stopMotor();
  }
}
