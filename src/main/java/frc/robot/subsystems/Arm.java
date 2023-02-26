// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  private RelativeEncoder armEncoder;

  private SparkMaxPIDController armSparkmaxPID;

  private SparkMaxAbsoluteEncoder alternateEncoder;

  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

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

    armSparkmaxPID.setOutputRange(-1, 1);

    alternateEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armSparkmaxPID.setFeedbackDevice(alternateEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Arm/Encoder", getArmEncoder());
    SmartDashboard.putNumber("/Arm/Bore", getBore());
  }

  public void controlArm(double speed) {
    armMotor.set(speed);
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public double getBore() {
    return alternateEncoder.getPosition();
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void setArmCommand(double setpoint, Rotation2d angle) {
    // armSparkmaxPID.setReference(setpoint, ControlType.kPosition);

    armMotor
        .getPIDController()
        .setReference(
            angle.getRadians(),
            ControlType.kPosition,
            0,
            feedforward.calculate(angle.getRadians(), 0));
  }

  public void resetEncoder() {
    armEncoder.setPosition(0.0);
  }
}
