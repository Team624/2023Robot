// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;

  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;

  private SparkMaxPIDController armSparkmaxPIDLeft;
  private SparkMaxPIDController armSparkmaxPIDRight;

  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

  private double RkP;
  private double RkI;
  private double RkD;

  private double LkP;
  private double LkI;
  private double LkD;

  public Arm() {

    armMotorRight = new CANSparkMax(frc.robot.Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.restoreFactoryDefaults();
    armEncoderRight = armMotorRight.getEncoder();
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armSparkmaxPIDRight = armMotorRight.getPIDController();

    RkP = frc.robot.Constants.Arm.RkP;
    RkI = frc.robot.Constants.Arm.RkI;
    RkD = frc.robot.Constants.Arm.RkD;

    armSparkmaxPIDRight.setP(RkP);
    armSparkmaxPIDRight.setI(RkI);
    armSparkmaxPIDRight.setD(RkD);

    armSparkmaxPIDRight.setOutputRange(-1, 1);

    armMotorLeft = new CANSparkMax(frc.robot.Constants.Arm.armMotorLeft, MotorType.kBrushless);
    armMotorLeft.restoreFactoryDefaults();
    armEncoderLeft = armMotorLeft.getEncoder();
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setInverted(true);
    armSparkmaxPIDLeft = armMotorLeft.getPIDController();

    LkP = frc.robot.Constants.Arm.LkP;
    LkI = frc.robot.Constants.Arm.LkI;
    LkD = frc.robot.Constants.Arm.LkD;

    armSparkmaxPIDLeft.setP(LkP);
    armSparkmaxPIDLeft.setI(LkI);
    armSparkmaxPIDLeft.setD(LkD);

    armSparkmaxPIDLeft.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Arm/Encoder/Left", getArmEncoderRight());
    SmartDashboard.putNumber("/Arm/Encoder/Right", getArmEncoderLeft());
  }

  public void controlArmRight(double speed) {
    armMotorRight.set(speed);
  }

  public void controlArmLeft(double speed) {
    armMotorLeft.set(speed);
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public void stopArm() {
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint, Rotation2d angle) {
    // armSparkmaxPID.setReference(setpoint, ControlType.kPosition);

    armMotorRight
        .getPIDController()
        .setReference(
            angle.getRadians(),
            ControlType.kPosition,
            0,
            feedforward.calculate(angle.getRadians(), 0));

    armMotorLeft
        .getPIDController()
        .setReference(
            angle.getRadians(),
            ControlType.kPosition,
            0,
            feedforward.calculate(angle.getRadians(), 0));
  }

  public void resetEncoder() {
    armEncoderRight.setPosition(0.0);
    armEncoderLeft.setPosition(0.0);
  }
}
