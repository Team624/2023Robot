// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  private RelativeEncoder armEncoder;

  private SparkMaxPIDController armPID;

  private double P;
  private double I;
  private double D;

  public Arm() {

    armMotor = new CANSparkMax(frc.robot.Constants.Arm.armMotor, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armEncoder = armMotor.getEncoder();
    armMotor.setIdleMode(IdleMode.kBrake);
    armPID = armMotor.getPIDController();

    P = frc.robot.Constants.Arm.P;
    I = frc.robot.Constants.Arm.I;
    D = frc.robot.Constants.Arm.D;

    armPID.setP(P);
    armPID.setI(I);
    armPID.setD(D);

    armPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void controlArm(double speed) {
    armMotor.set(speed);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }
}
