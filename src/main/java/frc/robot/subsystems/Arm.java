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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  private DutyCycleEncoder armBoreEncoder;

  private RelativeEncoder armEncoder;

  private ProfiledPIDController armController;

  private Rotation2d rotationReference;
  private Rotation2d currentRotation;

  private ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

  public Arm() {
    armMotor = new CANSparkMax(frc.robot.Constants.Arm.armMotor, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armEncoder = armMotor.getEncoder();
    armMotor.setIdleMode(IdleMode.kBrake);

    armController = new ProfiledPIDController(Constants.Arm.P, Constants.Arm.I, Constants.Arm.D, Constants.Arm.CONSTRAINTS);

    armBoreEncoder = new DutyCycleEncoder(Constants.Arm.BORE_ENCODER_DIO_PORT);

    rotationReference = getAbsoluteRotation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Arm/Encoder", getArmEncoder());
    SmartDashboard.putNumber("/Arm/Bore", getBore());
    SmartDashboard.putNumber("/Arm/BoreDegrees", getAbsoluteRotation().getDegrees());
    double ffradians = 2 * Math.PI - (-rotationReference.getRadians());
    System.out.println();
    double voltage = armController.calculate(/* getAbsoluteRotation().getRadians(), rotationReference.getRadians()) + */feedforward.calculate(2 * Math.PI - (-rotationReference.getRadians())
    , 0));

    System.out.println("Voltage: " + voltage);

    armMotor.setVoltage(voltage);
  }

  public void setReference(Rotation2d rotation) {
    this.rotationReference = rotation;
  }

  public void controlArm(double speed) {
    armMotor.set(speed);
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public double getBore() {
    return armBoreEncoder.getAbsolutePosition();
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBore();

    return new Rotation2d(radians);
  }

  public void stopArm() {
    rotationReference = getAbsoluteRotation();
    armMotor.stopMotor();
  }

  public void setArmCommand(double setpoint, Rotation2d angle) {
    // armSparkmaxPID.setReference(setpoint, ControlType.kPosition);

    armMotor.setVoltage(setpoint);
  }

  public void resetEncoder() {
    armEncoder.setPosition(0.0);
  }
}
