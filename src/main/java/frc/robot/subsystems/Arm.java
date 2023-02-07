// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  private TalonFX leftFlywheel;

  private RelativeEncoder armEncoder;

  private SparkMaxPIDController armSparkmaxPID;

  private PIDController armPID;
  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV, Constants.Arm.kA);

  private double P;
  private double I;
  private double D;

  public Arm() {

    armMotor = new CANSparkMax(frc.robot.Constants.Arm.armMotor, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armEncoder = armMotor.getEncoder();
    armMotor.setIdleMode(IdleMode.kBrake);
    armSparkmaxPID = armMotor.getPIDController();

    P = frc.robot.Constants.Arm.P;
    I = frc.robot.Constants.Arm.I;
    D = frc.robot.Constants.Arm.D;

    armSparkmaxPID.setP(P);
    armSparkmaxPID.setI(I);
    armSparkmaxPID.setD(D);

    // armPID.setP(P);
    // armPID.setI(I);
    // armPID.setD(D);

    armSparkmaxPID.setOutputRange(-1, 1);
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

  public void setArm1(double setpoint) {

    double VelocitySetPoint = 1.0;

    // double ffOutput = feedforward.calculate(setpoint, VelocitySetPoint);

    // double PIDOutput = armPID.calculate(armEncoder.getPosition(), setpoint);
    // armMotor.setVoltage(ffOutput);
    // armEncoder.setPosition(PIDOutput);
    // armMotor.set(PIDOutput);

    // armMotor.setVoltage(armPID.calculate(PIDOutput+ffOutput);

  }
}
