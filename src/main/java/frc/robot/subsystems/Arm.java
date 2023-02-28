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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;

  private ProfiledPIDController armController;

  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;

  private SparkMaxPIDController armSparkmaxPIDLeft;
  private SparkMaxPIDController armSparkmaxPIDRight;

  private Rotation2d rotationReference;

  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

  private double RkP;
  private double RkI;
  private double RkD;

  private double LkP;
  private double LkI;
  private double LkD;

  private DutyCycleEncoder armboreEncoder;

  public Arm() {

    armMotorRight = new CANSparkMax(frc.robot.Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.restoreFactoryDefaults();
    armEncoderRight = armMotorRight.getEncoder();
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armSparkmaxPIDRight = armMotorRight.getPIDController();
    armMotorRight.setCANTimeout(500);

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
    armMotorLeft.setCANTimeout(500);

    LkP = frc.robot.Constants.Arm.LkP;
    LkI = frc.robot.Constants.Arm.LkI;
    LkD = frc.robot.Constants.Arm.LkD;

    armSparkmaxPIDLeft.setP(LkP);
    armSparkmaxPIDLeft.setI(LkI);
    armSparkmaxPIDLeft.setD(LkD);

    armSparkmaxPIDLeft.setOutputRange(-1, 1);

    armboreEncoder = new DutyCycleEncoder(0);

    double maxVel = 0.0;
    double maxAcc = 0.0;

    armController =
        new ProfiledPIDController(
            Constants.Arm.RkP, Constants.Arm.RkI, Constants.Arm.RkD, Constants.Arm.CONSTRAINTS);

    rotationReference = getAbsoluteRotation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Arm/Encoder/Right", getArmEncoderRight());
    SmartDashboard.putNumber("/Arm/Encoder/Left", getArmEncoderLeft());
    SmartDashboard.putNumber("/Arm/BoreEncoder/get", armboreEncoder.get());
    SmartDashboard.putNumber("/Arm/BoreEncoder/Absolute", armboreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("/Arm/BoreEncoder/Distance", armboreEncoder.getDistance());

    // double ffradians = 2 * Math.PI - (-rotationReference.getRadians());

    // double voltage =
    //     armController.calculate(
    //         /* getAbsoluteRotation().getRadians(), rotationReference.getRadians()) + */
    // feedforward
    //             .calculate(2 * Math.PI - (-rotationReference.getRadians()), 0));

    // armMotorRight.setVoltage(voltage);
  }

  public void controlArmRight(double speed) {
    armMotorRight.set(speed);
  }

  public double getBoreEncoder() {
    return (1 - armboreEncoder.getAbsolutePosition());
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBoreEncoder();

    return new Rotation2d(radians);
  }

  public void controlArmLeft(double speed) {
    armMotorLeft.set(speed);
  }

  public void controlArm(double speed) {

    armMotorRight.set(speed);
    armMotorLeft.set(speed);
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public void stopArm() {
    rotationReference = getAbsoluteRotation();
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint) {
    System.out.println("setpoint in command: " + setpoint);
    Rotation2d angle = new Rotation2d(setpoint);
    // feedforward.calculate(angle.getRadians(), 0)

    armMotorLeft.setVoltage(setpoint);
    armMotorRight.setVoltage(setpoint);
  }

  public void ArmProfile(TrapezoidProfile.State setpoint) {

    System.out.println(setpoint.position);
    Rotation2d setpoint2d = new Rotation2d(setpoint.position);

    armSparkmaxPIDLeft.setReference(setpoint.position, ControlType.kPosition, 0);

    armSparkmaxPIDRight.setReference(setpoint.position, ControlType.kPosition, 0);
  }

  public void resetEncoder() {
    armEncoderRight.setPosition(0.0);
    armEncoderLeft.setPosition(0.0);
  }

  public void setReference(Rotation2d rotation) {
    this.rotationReference = rotation;
  }
}
