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

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor = new CANSparkMax(Constants.Pivot.PIVOT_MOTOR, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder;

  private SparkMaxPIDController pivotPID;
  private double P;
  private double I;
  private double D;
  private double Iz;
  private double FF;
  private double Pnew;
  private double Inew;
  private double Dnew;
  private double Iznew;
  private double FFnew;
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder = pivotMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void movePivot(double speed) {
    pivotMotor.set(speed);
  }

  public void pivotTo(double setPoint) {
    pivotPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void resetPivotEncoder() {
    pivotEncoder.setPosition(0);
  }

  public void stop() {
    pivotMotor.stopMotor();
  }
}
